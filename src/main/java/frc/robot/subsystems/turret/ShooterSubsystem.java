package frc.robot.subsystems.turret;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

  static final double kS = 0.16; // static friction (volts)
  static final double kV =
      0.104; // volts per RPS //0.10475 to tune use dutycycleout run it to 70 rps and kV =
  // voltage / // 0.116363636
  // 70 RPS
  static final double kA = 0.001; // volts per RPS^2

  static final double kP = 1.0;
  static final double kI = 0.0;
  static final double kD = 0.0;

  double kP_BOOST = 2.0; // ~2x normal
  double kD_BOOST = 0.002;

  static final double kP_SPIN_UP = 0.01; // 0.01
  static final double kI_SPIN_UP = 0.0; // 0.0001
  static final double kD_SPIN_UP = 0.0; // 0.0002

  private final SlewRateLimiter shooterRamp = new SlewRateLimiter(6); // volts per second
  double rampedSetpoint = 0.0;

  private final TalonFX shooterMotor = new TalonFX(1); // chjange ID
  private static final double GEAR_RATIO = 1.0; // 1:1

  private double timeAtSpeed = 0.0;
  private double lastRPM = 0.0;
  private boolean worked = false;
  SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(kS, kV, kA);
  PIDController velocityPID = new PIDController(kP, kI, kD);
  PIDController spinUpPID = new PIDController(kP_SPIN_UP, kI_SPIN_UP, kD_SPIN_UP);
  BangBangController bangBangController = new BangBangController();

  // config
  private final double overspinFactor = 1.0;
  private final double rpmRapidFireTolerance = 200;
  private final double rpmAccurateTolerance = 50;
  boolean shotsFired = false; // if shot recently fired then get the bang bang out
  private double lastTargetRPS = 0.0;
  private final VoltageOut voltageRequest = new VoltageOut(0);
  private double lastShotTime = 0.0;

  private static final InterpolatingDoubleTreeMap shotFlywheelSpeedMap =
      new InterpolatingDoubleTreeMap();

  static {
    // this is example later replace
    shotFlywheelSpeedMap.put(2.0, 4200.0); // its distance and second one is rpm
  }

  public ShooterSubsystem() {
    var config = new TalonFXConfiguration();
    config.MotorOutput.NeutralMode =
        NeutralModeValue
            .Coast; // if its breaking the it might fight the bangbang controller and fry stuff
    config.CurrentLimits.SupplyCurrentLimit = 40.0;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    shooterMotor.getConfigurator().apply(config);

    shooterRamp.reset(0.0);

    SmartDashboard.putNumber("Shooter/kP", kP);
    SmartDashboard.putNumber("Shooter/kD", kD);
    SmartDashboard.putNumber("Shooter/kV", kV);
  }

  @Override
  public void periodic() {
    double tuneP = SmartDashboard.getNumber("Shooter/kP", kP); // test
    double tuneD = SmartDashboard.getNumber("Shooter/kD", kD);

    double distance = getTargetDistance(); // change

    double baseRPM = shotFlywheelSpeedMap.get(distance);
    double targetRPM = baseRPM * overspinFactor;
    double currentRPM = getVelocityRevPerSec() * 60.0;

    double targetRPS = targetRPM / 60.0;
    double currentRPS = currentRPM / 60.0;

    //    rampedSetpoint = shooterRamp.calculate(setShooterRPM(targetRPS, currentRPS, spinUpPID));
    //    if (firstTime) {
    //      rampedSetpoint = 2.0;
    //      firstTime = false;
    //    }

    double rpmDrop = lastRPM - currentRPM;
    //    shotRecentlyFired = rpmDrop > 150; // tune
    double rpmSlope = (currentRPM - lastRPM) / 0.02;

    if (rpmSlope < -9500) {
      shotsFired = true;
      lastShotTime = currentTime.getAsDouble();
    }

    if (shotsFired && ((currentTime.getAsDouble() - lastShotTime) > 0.2)
        || currentRPS >= targetRPS - 20.0 / 60.0) {
      shotsFired = false;
    }

    lastRPM = currentRPM;
    switch (state) {
      case SPIN_UP:
        double voltage = setShooterRPM(targetRPS, currentRPS, spinUpPID);
        shooterMotor.setControl(voltageRequest.withOutput(voltage));
        break;
      case RAPID_FIRE:
        if (Math.abs(currentRPM - targetRPM) <= rpmRapidFireTolerance) {
          // feed balls
        } else {
          // dont feed
        }
        break;
      case RAPID_FIRE_ACCURATE:
        if (Math.abs(currentRPM - targetRPM) <= rpmAccurateTolerance) {
          // feed
        } else {
          // no no feed
        }
        break;
      case IDLE:
      case EMPTY:
        shooterMotor.setControl(voltageRequest.withOutput(0.0));
        break;
    }
    //    System.out.println(currentRPM + " " + shotsFired);
    //    System.out.println(
    //        "Shooter State: "
    //            + state
    //            + " Target RPM: "
    //            + targetRPM
    //            + " Current RPM: "
    //            //            + currentRPM
    //            + " Voltage Commanded: "
    //            + setShooterRPM(targetRPS, currentRPS, spinUpPID)
    //            + " Shot Recently Fired: "
    //            + shotRecentlyFired
    //            + " Atspeed: "
    //            + isAtspeed(targetRPS, currentRPS, rpmRapidFireTolerance));
  }

  double getVelocityRevPerSec() {
    double motorRPS = shooterMotor.getVelocity().getValueAsDouble();
    return (motorRPS / GEAR_RATIO);
  }

  public double setShooterRPM(double targetRPS, double currentRPS, PIDController PID) {
    //    double accelleration = (targetRPS - lastTargetRPS) / 0.02;
    double output = 0.0;
    if (shotsFired && currentRPS < targetRPS - (50.0 / 60.0)) {
      //      PID.setP(kP_BOOST);
      //      PID.setD(kD_BOOST);
      double bangBangVolts = bangBangController.calculate(currentRPS, targetRPS) * 4.0;
      double ffVolts = feedforward.calculate(targetRPS);
      // maybe add deadband here for osolation?
      output = ffVolts + bangBangVolts;
      System.out.println("BOOST \n BOOST \n BOOST \n BOOST \n BOOST \n BOOST \n BOOST \n BOOST ");
    } else {
      //      PID.setP(kP_SPIN_UP);
      //      PID.setD(kD_SPIN_UP);
      double ffVolts = feedforward.calculate(targetRPS);
      // maybe add deadband here for osolation?
      double pidVolts = PID.calculate(currentRPS, targetRPS);
      output = ffVolts + pidVolts;
    }

    lastTargetRPS = targetRPS;
    output = MathUtil.clamp(output, -12.0, 12.0);
    output = shooterRamp.calculate(output);
    return output;
  }

  public void setState(ShooterState newState) {
    state = newState;
    shooterRamp.reset(0.0);
  }

  private double getTargetDistance() {
    // find distance somehow idk prob pose
    return 2.0;
  }

  private boolean isAtSpeed(double targetRPS, double currentRPS, double toleranceRM) {
    return (Math.abs(currentRPS * 60.0 - targetRPS * 60.0) <= toleranceRM);
  }

  public void onEnable() {
    double currentRPS = getVelocityRevPerSec();
    shooterRamp.reset(shooterMotor.getMotorVoltage().getValueAsDouble());

    spinUpPID.reset();
    velocityPID.reset();

    //    bangBangController

    lastRPM = getVelocityRevPerSec() * 60.0;
    lastTargetRPS = 0.0;

    shotsFired = false;
  }
}
