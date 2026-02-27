package frc.robot.subsystems.shooting;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class HoodSubsystem extends SubsystemBase {

  private static final HoodSubsystem INSTANCE = new HoodSubsystem();

  public static HoodSubsystem getInstance() {
    return INSTANCE;
  }

  private static final int HOOD_MOTOR_ID = 22; // tune
  private static final double GEAR_RATIO = 6.0; // tune

  private static final double FORWARD_SOFT_LIMIT = 1.75; // tunemax hood angle
  private static final double REVERSE_SOFT_LIMIT = 0.0; // tune min hood angle

  private static final double kP = 15.0; // tune
  private static final double kI = 0.0;
  private static final double kD = 0.5; // tune
  private static final double kG = 0.3; // tune gravity
  private static final double kS = 0.1; // tune

  private final TalonFX hoodMotor = new TalonFX(HOOD_MOTOR_ID);
  private final PositionVoltage positionRequest = new PositionVoltage(0);

  private static final InterpolatingDoubleTreeMap hoodAngleMap = new InterpolatingDoubleTreeMap();

  static {
    // distance in meters and   hood position in rotations
    hoodAngleMap.put(
        Units.inchesToMeters(74.5 + 27 / 2 + 23.25), 0.0); // distance in meters, time in seconds
    hoodAngleMap.put(Units.inchesToMeters(84.5 + 27 / 2 + 23.25), 0.0);
    hoodAngleMap.put(Units.inchesToMeters(94.5 + 27 / 2 + 23.25), 0.0);
    hoodAngleMap.put(Units.inchesToMeters(104.5 + 27 / 2 + 23.25), 0.0);
    hoodAngleMap.put(Units.inchesToMeters(114.5 + 27 / 2 + 23.25), 0.0);
    hoodAngleMap.put(Units.inchesToMeters(124.5 + 27 / 2 + 23.25), 0.0);
    hoodAngleMap.put(Units.inchesToMeters(134.5 + 27 / 2 + 23.25), 0.0);
    hoodAngleMap.put(Units.inchesToMeters(144.5 + 27 / 2 + 23.25), 0.0);
    hoodAngleMap.put(Units.inchesToMeters(166.5 + 27 / 2 + 23.25), 60.0);
  }

  private double targetPositionRotations = 0.0;

  private HoodSubsystem() {
    //    SmartDashboard.putNumber("Target Angle", 0.0);
    var config = new TalonFXConfiguration();
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive; // tune

    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = 30.0;
    config.CurrentLimits.StatorCurrentLimit = 60.0;
    config.CurrentLimits.StatorCurrentLimitEnable = true;

    config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = FORWARD_SOFT_LIMIT;
    config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = REVERSE_SOFT_LIMIT;

    config.Slot0.kP = kP;
    config.Slot0.kI = kI;
    config.Slot0.kD = kD;
    config.Slot0.kG = kG;
    config.Slot0.kS = kS;
    config.Slot0.GravityType = GravityTypeValue.Arm_Cosine;

    config.Feedback.SensorToMechanismRatio = GEAR_RATIO;

    hoodMotor.getConfigurator().apply(config);
    hoodMotor.setPosition(0);
  }

  @Override
  public void periodic() {
    hoodMotor.setControl(positionRequest.withPosition(targetPositionRotations));
  }

  public void setTargetDistance(double distanceMeters) {
    targetPositionRotations = hoodAngleMap.get(distanceMeters);
    //    targetPositionRotations =
    //        SmartDashboard.getNumber("Target Angle", 0.0) / 360.0; // convert from degrees to
    // rotations
  }

  public void setPosition(double positionRotations) {
    targetPositionRotations = positionRotations;
  }

  public double getPosition() {
    return hoodMotor.getPosition().getValueAsDouble();
  }

  public boolean atTarget() {
    return Math.abs(getPosition() - targetPositionRotations) < 30.0 / 360.0; // tune
  }
}
