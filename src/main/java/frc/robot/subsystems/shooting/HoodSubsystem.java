package frc.robot.subsystems.shooting;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.FieldConstants;
import frc.robot.util.geometry.AllianceFlipUtil;
import org.littletonrobotics.junction.Logger;

public class HoodSubsystem extends SubsystemBase {

  private static final HoodSubsystem INSTANCE;

  public static HoodSubsystem getInstance() {
    return INSTANCE;
  }

  private static final int HOOD_MOTOR_ID = 22; // tune
  private static final double GEAR_RATIO = 6.0; // tune

  private static final double FORWARD_SOFT_LIMIT = 2.0; // tunemax hood angle
  private static final double REVERSE_SOFT_LIMIT = 0.0; // tune min hood angle

  private static final double kP = 10.0; // tune
  private static final double kI = 0.0;
  private static final double kD = 0.5; // tune
  private static final double kG = 0.3; // tune gravity
  private static final double kS = 0.1; // tune

  private final TalonFX hoodMotor = new TalonFX(HOOD_MOTOR_ID);
  private final PositionVoltage positionRequest = new PositionVoltage(0).withEnableFOC(true);

  private static final InterpolatingDoubleTreeMap hoodAngleMap = new InterpolatingDoubleTreeMap();

  private double robotPoseX = 0;
  private double targetDistance = 0.0;
  private boolean trenchMode = false;

  // ── LIVE TRIM ─────────────────────────────────────────────────────────────
  private double globalHoodTrim = 0.0;
  private final InterpolatingDoubleTreeMap hoodTrimOverlay = new InterpolatingDoubleTreeMap();

  private static final double[] DISTANCE_KEYS = {
    1.320, 1.573, 1.723, 2.002, 2.250, 2.514, 2.750, 3.000, 3.250, 3.500, 3.730, 4.000, 4.250,
    4.500, 4.750, 5.000, 5.250, 5.500, 5.750, 6.000, 6.250
  };

  static {
    // distance (m) -> hood position (rotations)
    hoodAngleMap.put(1.320, 0.0);
    hoodAngleMap.put(1.573, 0.0);
    hoodAngleMap.put(1.723, 0.0);
    hoodAngleMap.put(2.002, 0.0);
    hoodAngleMap.put(2.250, 0.0);
    hoodAngleMap.put(2.514, 30.0 / 360.0);
    hoodAngleMap.put(2.750, 40.0 / 360.0);
    hoodAngleMap.put(3.000, 50.0 / 360.0);
    hoodAngleMap.put(3.250, 120.0 / 360.0);
    hoodAngleMap.put(3.500, 150.0 / 360.0);
    hoodAngleMap.put(3.730, 175.0 / 360.0);
    hoodAngleMap.put(4.000, 180.0 / 360.0);
    hoodAngleMap.put(4.250, 180.0 / 360.0);
    hoodAngleMap.put(4.500, 220.0 / 360.0);
    hoodAngleMap.put(4.750, 230.0 / 360.0);
    hoodAngleMap.put(5.000, 240.0 / 360.0);
    hoodAngleMap.put(5.250, 260.0 / 360.0);
    hoodAngleMap.put(5.500, 280.0 / 360.0);
    hoodAngleMap.put(5.750, 300.0 / 360.0);
    hoodAngleMap.put(6.000, 300.0 / 360.0);
    hoodAngleMap.put(6.250, 300.0 / 360.0);
    INSTANCE = new HoodSubsystem();
  }

  private double targetPositionRotations = 0.0;

  private HoodSubsystem() {
    for (double key : DISTANCE_KEYS) {
      hoodTrimOverlay.put(key, 0.0);
    }
    SmartDashboard.putNumber("Target Angle", 0.0);
    var config = new TalonFXConfiguration();
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive; // tune

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
    com.ctre.phoenix6.hardware.ParentDevice.optimizeBusUtilizationForAll(hoodMotor);
  }

  public boolean isTrenchMode() {
    return trenchMode;
  }

  @Override
  public void periodic() {
    double allianceX = AllianceFlipUtil.applyX(robotPoseX);
    trenchMode = false;

    if (allianceX > 2 && allianceX < 7) {
      hoodMotor.setControl(positionRequest.withPosition(0.0));
      trenchMode = true;
    } else if (allianceX < FieldConstants.FIELDLENGTH - 2.5
        && allianceX > FieldConstants.FIELDLENGTH - 6.5) {
      hoodMotor.setControl(positionRequest.withPosition(0.0));
      trenchMode = true;
    } else if (allianceX > 6.5) {
      hoodMotor.setControl(
          positionRequest.withPosition(
              FORWARD_SOFT_LIMIT
                  - 0.05)); // give it a little buffer so it doesn't hit the soft stop
    } else {
      hoodMotor.setControl(positionRequest.withPosition(targetPositionRotations));
      //      hoodMotor.setControl(
      //          positionRequest.withPosition(SmartDashboard.getNumber("Target Angle", 0.0) /
      // 360.0));
    }

    Logger.recordOutput("Hood/RobotPoseX", robotPoseX);
    Logger.recordOutput("Hood/AllianceRelativeX", allianceX);
    Logger.recordOutput("Hood/TrenchMode", trenchMode);
    Logger.recordOutput("Hood/GlobalHoodTrim", globalHoodTrim);
    Logger.recordOutput("Hood/PerDistHoodTrim", hoodTrimOverlay.get(targetDistance));

    // if shooter is tracking human tehe then
    //    hoodMotor.setControl(positionRequest.withPosition(FORWARD_SOFT_LIMIT - 0.05));
  }

  public void setTargetDistance(double distanceMeters) {
    targetDistance = distanceMeters;
    targetPositionRotations =
        MathUtil.clamp(
            hoodAngleMap.get(distanceMeters) + hoodTrimOverlay.get(distanceMeters) + globalHoodTrim,
            0.0,
            FORWARD_SOFT_LIMIT);
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
    return Math.abs(getPosition() - targetPositionRotations) < 50.0 / 360.0; // tune
  }

  // ── TRIM HELPERS ───────────────────────────────────────────────────────
  public double getTargetDistance() {
    return targetDistance;
  }

  private double getNearestKey(double distance) {
    double nearest = DISTANCE_KEYS[0];
    double minDiff = Math.abs(distance - nearest);
    for (double key : DISTANCE_KEYS) {
      double diff = Math.abs(distance - key);
      if (diff < minDiff) {
        minDiff = diff;
        nearest = key;
      }
    }
    return nearest;
  }

  public void adjustHoodTrimAtDistance(double distance, double delta) {
    double key = getNearestKey(distance);
    hoodTrimOverlay.put(key, hoodTrimOverlay.get(key) + delta);
  }

  public void adjustGlobalHoodTrim(double delta) {
    globalHoodTrim += delta;
  }

  public void resetHoodTrim() {
    globalHoodTrim = 0.0;
    for (double key : DISTANCE_KEYS) {
      hoodTrimOverlay.put(key, 0.0);
    }
  }

  public double getGlobalHoodTrim() {
    return globalHoodTrim;
  }
}
