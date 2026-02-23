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
import frc.robot.subsystems.handoff.HandoffSubsystem;

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
    PASSING,
    EMPTY
  }

  private ShooterState state = ShooterState.IDLE;

  // holding speed
  static final double kS = 0.16;
  static final double kV = 0.104;
  static final double kA = 0.001;
  static final double kP_STEADY = 0.1; // tune
  static final double kD_STEADY = 0.0;

  // getting back to speed after shot
  static final double kP_RECOVERY = 0.8; // tune — aggressive
  static final double kD_RECOVERY = 0.01; // tune — dampen overshoot

  // rpm error to switch slots
  static final double RECOVERY_THRESHOLD_RPS = 50.0 / 60.0; // low since flywheel drop is small

  // delay from handoff beam break
  private static final double BEAM_BREAK_DELAY_SECONDS = 0.15; // tune: travel time to flywheel
  private double beamBreakTriggerTime = -1.0;
  private boolean beamBreakPredicting = false;

  private final TalonFX shooterMotor = new TalonFX(1); // change ID
  private final TalonFX shooterMotor2 = new TalonFX(2); // change ID
  private static final double GEAR_RATIO = 1.0;

  // config
  private final double overspinFactor = 1.07;
  private final double rpmRapidFireTolerance = 200;
  private final double rpmAccurateTolerance = 50;

  private final SlewRateLimiter spinUpRamp = new SlewRateLimiter(40); // rps

  private final VelocityVoltage velocityRequest = new VelocityVoltage(0);
  private final VoltageOut stopRequest = new VoltageOut(0);

  private static final InterpolatingDoubleTreeMap shotFlywheelSpeedMap =
      new InterpolatingDoubleTreeMap();

  static {
    // example — replace with real data
    shotFlywheelSpeedMap.put(2.0, 4200.0); // distance (m) -> rpm
  }

  private ShooterSubsystem() {
    var config = new TalonFXConfiguration();
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    // 70a burst for 1.5s drop to 35a sustained
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = 70.0;
    config.CurrentLimits.SupplyCurrentLowerLimit = 35.0;
    config.CurrentLimits.SupplyCurrentLowerTime = 1.5;
    config.CurrentLimits.StatorCurrentLimit = 120.0;
    config.CurrentLimits.StatorCurrentLimitEnable = true;

    config.Slot0.kS = kS;
    config.Slot0.kV = kV;
    config.Slot0.kA = kA;
    config.Slot0.kP = kP_STEADY;
    config.Slot0.kI = 0.0;
    config.Slot0.kD = kD_STEADY;

    config.Slot1.kS = kS;
    config.Slot1.kV = kV;
    config.Slot1.kA = kA;
    config.Slot1.kP = kP_RECOVERY;
    config.Slot1.kI = 0.0;
    config.Slot1.kD = kD_RECOVERY;

    shooterMotor.getConfigurator().apply(config);

    // follower
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
    double targetRPM;
    if (targetRPMOverride > 0) {
      targetRPM = targetRPMOverride;
    } else {
      double distance = getTargetDistance();
      targetRPM = shotFlywheelSpeedMap.get(distance) * overspinFactor;
    }
    double currentRPM = getVelocityRevPerSec() * 60.0;

    double targetRPS = targetRPM / 60.0;
    double currentRPS = currentRPM / 60.0;

    double now = Timer.getFPGATimestamp();
    boolean handoffBeamBroken = HandoffSubsystem.getInstance().isBeamBroken();

    if (handoffBeamBroken && beamBreakTriggerTime < 0) {
      beamBreakTriggerTime = now;
    } else if (!handoffBeamBroken) {
      beamBreakTriggerTime = -1.0;
    }

    beamBreakPredicting =
        beamBreakTriggerTime > 0 && (now - beamBreakTriggerTime) >= BEAM_BREAK_DELAY_SECONDS;

    switch (state) {
      case SPIN_UP:
        double rampedRPS = spinUpRamp.calculate(targetRPS);
        shooterMotor.setControl(velocityRequest.withVelocity(rampedRPS).withSlot(0));
        break;
      case RAPID_FIRE:
      case RAPID_FIRE_ACCURATE:
        // slot 1 if beam break or rpm error exceeds threshold
        double error = Math.abs(targetRPS - currentRPS);
        boolean useRecovery = beamBreakPredicting || error > RECOVERY_THRESHOLD_RPS;
        int slot = useRecovery ? 1 : 0;
        shooterMotor.setControl(velocityRequest.withVelocity(targetRPS).withSlot(slot));
        break;
      case PASSING:
        shooterMotor.setControl(stopRequest.withOutput(12.0)); // full voltage for passing
        break;
      case IDLE:
      case EMPTY:
        shooterMotor.setControl(stopRequest.withOutput(0.0));
        spinUpRamp.reset(0.0);
        beamBreakTriggerTime = -1.0;
        beamBreakPredicting = false;
        break;
    }
  }

  double getVelocityRevPerSec() {
    double motorRPS = shooterMotor.getVelocity().getValueAsDouble();
    return (motorRPS / GEAR_RATIO);
  }

  public double getVelocityRPM() {
    return getVelocityRevPerSec() * 60.0;
  }

  public void setState(ShooterState newState) {
    if (newState == ShooterState.SPIN_UP && state == ShooterState.IDLE) {
      spinUpRamp.reset(getVelocityRevPerSec());
    }
    state = newState;
  }

  public ShooterState getState() {
    return state;
  }

  private double getEffectiveTargetRPM() {
    if (targetRPMOverride > 0) {
      return targetRPMOverride;
    }
    return shotFlywheelSpeedMap.get(getTargetDistance()) * overspinFactor;
  }

  public boolean isAtGoalSpeed() {
    return Math.abs(getVelocityRPM() - getEffectiveTargetRPM()) <= rpmRapidFireTolerance;
  }

  public boolean isAtGoalSpeedAccurate() {
    return Math.abs(getVelocityRPM() - getEffectiveTargetRPM()) <= rpmAccurateTolerance;
  }

  private double targetDistance = 0.0;
  private double targetRPMOverride = -1.0; // negative = use map, positive = use this RPM

  public void setTargetDistance(double distance) {
    targetDistance = distance;
  }

  /** Override target RPM directly (for tuning). Set to -1 to use distance-based map. */
  public void setTargetRPMOverride(double rpm) {
    targetRPMOverride = rpm;
  }

  private double getTargetDistance() {
    return targetDistance;
  }

  public void onEnable() {
    spinUpRamp.reset(getVelocityRevPerSec());
  }
}
