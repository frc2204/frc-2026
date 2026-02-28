package frc.robot.subsystems.shooting;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.geometry.AllianceFlipUtil;

public class HoodSubsystem extends SubsystemBase {

  private static final HoodSubsystem INSTANCE = new HoodSubsystem();

  public static HoodSubsystem getInstance() {
    return INSTANCE;
  }

  private static final int HOOD_MOTOR_ID = 22; // tune
  private static final double GEAR_RATIO = 6.0; // tune

  private static final double FORWARD_SOFT_LIMIT = 1.75; // tunemax hood angle
  private static final double REVERSE_SOFT_LIMIT = 0.0; // tune min hood angle

  private static final double kP = 10.0; // tune
  private static final double kI = 0.0;
  private static final double kD = 0.5; // tune
  private static final double kG = 0.3; // tune gravity
  private static final double kS = 0.1; // tune

  private final TalonFX hoodMotor = new TalonFX(HOOD_MOTOR_ID);
  private final PositionVoltage positionRequest = new PositionVoltage(0);

  private static final InterpolatingDoubleTreeMap hoodAngleMap = new InterpolatingDoubleTreeMap();

  private double robotPoseX = 0;

  static {
    // distance in meters and   hood position in rotations
    hoodAngleMap.put(Units.inchesToMeters(30.0 + 27 / 2 + 23.25 + 3.5), 0.0);
    hoodAngleMap.put(Units.inchesToMeters(40.0 + 27 / 2 + 23.25 + 3.5), 0.0);
    hoodAngleMap.put(Units.inchesToMeters(50.0 + 27 / 2 + 23.25 + 3.5), 0.0);
    hoodAngleMap.put(Units.inchesToMeters(60.0 + 27 / 2 + 23.25 + 3.5), 60.0 / 360);
    hoodAngleMap.put(Units.inchesToMeters(70.0 + 27 / 2 + 23.25 + 3.5), 70.0 / 360);
    hoodAngleMap.put(Units.inchesToMeters(80.0 + 27 / 2 + 23.25 + 3.5), 80.0 / 360);
    hoodAngleMap.put(Units.inchesToMeters(130), (170.0 - 50.0) / 360);
    hoodAngleMap.put(Units.inchesToMeters(140), (170.0 - 50.0) / 360);
    hoodAngleMap.put(Units.inchesToMeters(150), (200.0 - 50.0) / 360);
    hoodAngleMap.put(Units.inchesToMeters(160), (200.0 - 50.0) / 360);
    hoodAngleMap.put(Units.inchesToMeters(170), (210.0 - 50.0) / 360);
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
    if (AllianceFlipUtil.applyX(robotPoseX) > 3.500
        && AllianceFlipUtil.applyX(robotPoseX) < 5.500) {
      hoodMotor.setControl(
          positionRequest.withPosition(
              0.0)); // lolwer hood when  approaching trench TODO: make it only go down when
      // robottoturret, not just robot pose, and tune
    } else if (AllianceFlipUtil.applyX(robotPoseX) > 5.500) {
      hoodMotor.setControl(positionRequest.withPosition(FORWARD_SOFT_LIMIT - 0.05)); // give it a little buffer so it doesn't hit the soft stop
    } else {
      hoodMotor.setControl(positionRequest.withPosition(targetPositionRotations));
    }
  }

  public void setTargetDistance(double distanceMeters) {
    targetPositionRotations = hoodAngleMap.get(distanceMeters);
    //    targetPositionRotations =
    //        SmartDashboard.getNumber("Target Angle", 0.0) / 360.0; // convert from degrees to
    // rotations
  }

  public void setRobotPosex(double xMeters) {
    robotPoseX = xMeters;
  }

  public void setPosition(double positionRotations) {
    targetPositionRotations = positionRotations;
  }

  public double getPosition() {
    return hoodMotor.getPosition().getValueAsDouble();
  }

  public boolean atTarget() {
    return Math.abs(getPosition() - targetPositionRotations) < 10.0 / 360.0; // tune
  }
}
