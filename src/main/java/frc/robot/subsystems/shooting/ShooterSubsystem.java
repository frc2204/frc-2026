package frc.robot.subsystems.shooting;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;

public class ShooterSubsystem extends SubsystemBase {
  private static final ShooterSubsystem INSTANCE = new ShooterSubsystem();

  public static ShooterSubsystem getInstance() {
    return INSTANCE;
  }

  public enum ShooterState {
    IDLE,
    SPIN_UP,
    RAPID_FIRE,
    RAPID_FIRE_ACCURATE,
    EMPTY
  }

  private ShooterState state = ShooterState.IDLE;

  private final DoubleSupplier currentTime = Timer::getFPGATimestamp;

  // Slot 0: steady-state (holding speed)
  static final double kS = 0.16;
  static final double kV = 0.104;
  static final double kA = 0.001;
  static final double kP_STEADY = 0.1; // tune
  static final double kD_STEADY = 0.0;

  // Slot 1: recovery (getting back to speed after shot)
  static final double kP_RECOVERY = 0.8; // tune — aggressive
  static final double kD_RECOVERY = 0.01; // tune — dampen overshoot

  // RPM error threshold to switch slots
  static final double RECOVERY_THRESHOLD_RPS = 200.0 / 60.0;

  private final TalonFX shooterMotor = new TalonFX(1); // change ID
  private final TalonFX shooterMotor2 = new TalonFX(2); // change ID
  private static final double GEAR_RATIO = 1.0;

  private double lastRPM = 0.0;
  private boolean shotsFired = false;
  private double lastShotTime = 0.0;

  // Config
  private final double overspinFactor = 1.07;
  private final double rpmRapidFireTolerance = 200;
  private final double rpmAccurateTolerance = 50;

  // Spin-up slew rate (RPS per second) — prevents huge current spike from 0 to target
  private final SlewRateLimiter spinUpRamp = new SlewRateLimiter(40); // tune: RPS/s

  // Control requests — reused each cycle
  private final VelocityVoltage velocityRequest = new VelocityVoltage(0).withEnableFOC(true);
  private final VoltageOut stopRequest = new VoltageOut(0);

  private static final InterpolatingDoubleTreeMap shotFlywheelSpeedMap =
      new InterpolatingDoubleTreeMap();

  static {
    // example — replace with real data
    shotFlywheelSpeedMap.put(2.0, 4200.0); // distance (m) -> rpm
  }

  public ShooterSubsystem() {
    var config = new TalonFXConfiguration();
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    // Current limits: 70A burst for 1.5s, drop to 35A sustained
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = 70.0;
    config.CurrentLimits.SupplyCurrentLowerLimit = 35.0;
    config.CurrentLimits.SupplyCurrentLowerTime = 1.5;
    config.CurrentLimits.StatorCurrentLimit = 120.0;
    config.CurrentLimits.StatorCurrentLimitEnable = true;

    // Slot 0: steady-state — soft PID, same FF
    config.Slot0.kS = kS;
    config.Slot0.kV = kV;
    config.Slot0.kA = kA;
    config.Slot0.kP = kP_STEADY;
    config.Slot0.kI = 0.0;
    config.Slot0.kD = kD_STEADY;

    // Slot 1: recovery — aggressive PID, same FF
    config.Slot1.kS = kS;
    config.Slot1.kV = kV;
    config.Slot1.kA = kA;
    config.Slot1.kP = kP_RECOVERY;
    config.Slot1.kI = 0.0;
    config.Slot1.kD = kD_RECOVERY;

    shooterMotor.getConfigurator().apply(config);

    // Follower motor — same current limits
    var config2 = new TalonFXConfiguration();
    config2.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    config2.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    config2.CurrentLimits.SupplyCurrentLimitEnable = true;
    config2.CurrentLimits.SupplyCurrentLimit = 70.0;
    config2.CurrentLimits.SupplyCurrentLowerLimit = 35.0;
    config2.CurrentLimits.SupplyCurrentLowerTime = 1.5;
    config2.CurrentLimits.StatorCurrentLimit = 120.0;
    config2.CurrentLimits.StatorCurrentLimitEnable = true;
    shooterMotor2.getConfigurator().apply(config2);
    shooterMotor2.setControl(new Follower(shooterMotor.getDeviceID(), MotorAlignmentValue.Opposed));
  }

  @Override
  public void periodic() {
    double distance = getTargetDistance();

    double baseRPM = shotFlywheelSpeedMap.get(distance);
    double targetRPM = baseRPM * overspinFactor;
    double currentRPM = getVelocityRevPerSec() * 60.0;

    double targetRPS = targetRPM / 60.0;
    double currentRPS = currentRPM / 60.0;

    // Detect shot fired via RPM slope
    double rpmSlope = (currentRPM - lastRPM) / 0.02;
    if (rpmSlope < -9500) {
      shotsFired = true;
      lastShotTime = currentTime.getAsDouble();
    }
    if (shotsFired
        && ((currentTime.getAsDouble() - lastShotTime) > 0.2
            || currentRPS >= targetRPS - 20.0 / 60.0)) {
      shotsFired = false;
    }

    lastRPM = currentRPM;

    switch (state) {
      case SPIN_UP:
        // Ramp up gradually to prevent huge current spike
        double rampedRPS = spinUpRamp.calculate(targetRPS);
        shooterMotor.setControl(velocityRequest.withVelocity(rampedRPS).withSlot(0));
        break;
      case RAPID_FIRE:
      case RAPID_FIRE_ACCURATE:
        // No ramp — go straight to target, use recovery slot when far from setpoint
        double error = Math.abs(targetRPS - currentRPS);
        int slot = error > RECOVERY_THRESHOLD_RPS ? 1 : 0;
        shooterMotor.setControl(velocityRequest.withVelocity(targetRPS).withSlot(slot));
        break;
      case IDLE:
      case EMPTY:
        shooterMotor.setControl(stopRequest.withOutput(0.0));
        spinUpRamp.reset(0.0);
        break;
    }
  }

  double getVelocityRevPerSec() {
    double motorRPS = shooterMotor.getVelocity().getValueAsDouble();
    return (motorRPS / GEAR_RATIO);
  }

  public void setState(ShooterState newState) {
    if (newState == ShooterState.SPIN_UP && state == ShooterState.IDLE) {
      spinUpRamp.reset(getVelocityRevPerSec()); // start ramp from current speed
    }
    state = newState;
  }

  public ShooterState getState() {
    return state;
  }

  public boolean isAtGoalSpeed() {
    double distance = getTargetDistance();
    double targetRPM = shotFlywheelSpeedMap.get(distance) * overspinFactor;
    double currentRPM = getVelocityRevPerSec() * 60.0;
    return Math.abs(currentRPM - targetRPM) <= rpmRapidFireTolerance;
  }

  public boolean isAtGoalSpeedAccurate() {
    double distance = getTargetDistance();
    double targetRPM = shotFlywheelSpeedMap.get(distance) * overspinFactor;
    double currentRPM = getVelocityRevPerSec() * 60.0;
    return Math.abs(currentRPM - targetRPM) <= rpmAccurateTolerance;
  }

  private double getTargetDistance() {
    // find distance somehow idk prob pose
    return 2.0;
  }

  public void onEnable() {
    lastRPM = getVelocityRevPerSec() * 60.0;
    shotsFired = false;
    spinUpRamp.reset(getVelocityRevPerSec());
  }
}
