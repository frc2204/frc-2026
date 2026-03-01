package frc.robot.subsystems.shooting;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.util.FieldConstants;
import frc.robot.util.geometry.AllianceFlipUtil;
import java.util.function.Supplier;

// TODO: make limelight update the pose, but reject bad data thats too far away, start making
// shooting on the move
// TODO: limit speed, closer we are to target slower we go, farther we are the less limited and
// maybe remove bang bang and also add the phase shifting thing and shoot balls a lil earlier
// because takes a bit to go into hub and also to detect

public class TurretSubsystem extends SubsystemBase {

  private static double VISION_LATENCY;

  private final NetworkTable limelightTable;
  private Translation2d targetPose = new Translation2d();
  private boolean priorityIdSet = false;
  private static final double VISION_KP = 0.015;
  private Pose2d robotPose;
  private final Supplier<Pose2d> poseSupplier;
  private final Supplier<ChassisSpeeds> chassisSpeedsSupplier;

  private Translation2d robotFieldVelocity;

  private static final Translation2d robotToTurret =
      new Translation2d(Units.inchesToMeters(-8.5), 0);

  private final TalonFX turretMotor = new TalonFX(20);
  private final MotionMagicVoltage motionMagicRequest = new MotionMagicVoltage(0);

  private static final double MAX_ANGLE = 90;
  private static final double MIN_ANGLE = -90;
  private static final double TURRET_FORWARD_OFFSET_RAD = Math.PI;
  private static final double MAX_VELOCITY_IN_DEG_PER_SEC = 35000;
  private static final double MAX_ACCELERATION_IN_DEG_PER_SEC = 1000;
  private static final double UNWIND_THRESHOLD = 500;
  private static final double GEAR_RATIO = 50.0;

  private Rotation2d targetAngle;
  private boolean isUnwinding;
  private double lastUnwindTime = Double.POSITIVE_INFINITY;

  private boolean wrappingAround;
  private boolean atSoftLimit = false;

  private double lastTimeAtGoal = 0;

  private static final InterpolatingDoubleTreeMap ballFlightTimeMap =
      new InterpolatingDoubleTreeMap();

  static {
    // placeholder arc-style flight times — measure and replace
    // Original values commented out:
    // ballFlightTimeMap.put(Units.inchesToMeters(74.5 + 27.0 / 2.0 + 23.25), 1.0);
    // ballFlightTimeMap.put(Units.inchesToMeters(84.5 + 27.0 / 2.0 + 23.25), 1.0);
    // ballFlightTimeMap.put(Units.inchesToMeters(94.5 + 27.0 / 2.0 + 23.25), 1.1);
    // ballFlightTimeMap.put(Units.inchesToMeters(104.5 + 27.0 / 2.0 + 23.25), 1.1);
    // ballFlightTimeMap.put(Units.inchesToMeters(114.5 + 27.0 / 2.0 + 23.25), 1.2);
    // ballFlightTimeMap.put(Units.inchesToMeters(124.5 + 27.0 / 2.0 + 23.25), 1.2);
    // ballFlightTimeMap.put(Units.inchesToMeters(134.5 + 27.0 / 2.0 + 23.25), 1.2);
    // ballFlightTimeMap.put(Units.inchesToMeters(144.5 + 27.0 / 2.0 + 23.25), 1.3);
    // ballFlightTimeMap.put(Units.inchesToMeters(166.5 + 27.0 / 2.0 + 23.25), 1.3);
    ballFlightTimeMap.put(2.5, 0.30 * 2); // distance (m) -> flight time (s)
    ballFlightTimeMap.put(2.8, 0.35 * 2);
    ballFlightTimeMap.put(3.1, 0.40 * 2);
    ballFlightTimeMap.put(3.4, 0.45 * 2);
    ballFlightTimeMap.put(3.7, 0.50 * 2);
    ballFlightTimeMap.put(4.0, 0.55 * 2);
    ballFlightTimeMap.put(4.3, 0.60 * 2);
    ballFlightTimeMap.put(4.6, 0.65 * 2);
    ballFlightTimeMap.put(5.3, 0.75 * 2);
  }

  public TurretSubsystem(
      Supplier<Pose2d> poseSupplier, Supplier<ChassisSpeeds> chassisSpeedsSupplier) {
    this.poseSupplier = poseSupplier;
    this.chassisSpeedsSupplier = chassisSpeedsSupplier;
    limelightTable = NetworkTableInstance.getDefault().getTable("limelight-three");

    this.targetAngle = new Rotation2d();
    this.isUnwinding = false;

    var config = new TalonFXConfiguration();
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    config.CurrentLimits.SupplyCurrentLimit = 40.0;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;

    config.Feedback.SensorToMechanismRatio = GEAR_RATIO;

    config.MotionMagic.MotionMagicCruiseVelocity = MAX_VELOCITY_IN_DEG_PER_SEC / 360.0; // 97.22 rps
    config.MotionMagic.MotionMagicAcceleration =
        MAX_ACCELERATION_IN_DEG_PER_SEC / 360.0; // 2.78 rps^2

    config.Slot0.kP = 60.0; // lower by a lot maybe
    config.Slot0.kI = 0.0;
    config.Slot0.kD = 0.63;
    config.Slot0.kS = 0.2;
    config.Slot0.kV = 0.754;
    config.Slot0.kA = 0.126;

    config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = MAX_ANGLE / 360.0; // 2.0 rotations
    config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = MIN_ANGLE / 360.0; // -2.0 rotations

    turretMotor.getConfigurator().apply(config);
    turretMotor.setPosition(0 / 360.0);
    com.ctre.phoenix6.hardware.ParentDevice.optimizeBusUtilizationForAll(turretMotor);
  }

  @Override
  public void periodic() {
    // Set limelight priority ID once alliance is known
    if (!priorityIdSet && edu.wpi.first.wpilibj.DriverStation.getAlliance().isPresent()) {
      if (AllianceFlipUtil.shouldFlip()) {
        limelightTable.getEntry("priorityid").setNumber(25);
      } else {
        limelightTable.getEntry("priorityid").setNumber(10);
      }
      priorityIdSet = true;
    }

    LimelightHelpers.setCameraPose_RobotSpace(
        "limelight",
        0.0127, // x meters forward
        -0.1143, // y meters left
        0, // z meters up
        0, // roll deg
        0, // pitch deg
        getAbsolutePositionDeg() + 90 // yaw deg
        );
    VISION_LATENCY =
        (limelightTable.getEntry("tl").getDouble(0.0)
                + limelightTable.getEntry("cl").getDouble(0.0))
            / 1000.0;
    robotPose = poseSupplier.get();
    ChassisSpeeds chassisSpeeds = chassisSpeedsSupplier.get();
    robotFieldVelocity =
        new Translation2d(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond)
            .rotateBy(robotPose.getRotation());

    double currentPositionDeg = getAbsolutePositionDeg();

    // Update shooter and hood with current distance to hub
    double distanceToHub = getDistanceFromHub(robotPose);
    ShooterSubsystem.getInstance().setTargetDistance(distanceToHub);
    HoodSubsystem.getInstance().setTargetDistance(distanceToHub);
    //    System.out.println(Units.metersToInches(distanceToHub));
    Translation2d turretFieldPos =
        robotPose.getTranslation().plus(robotToTurret.rotateBy(robotPose.getRotation()));
    //    HoodSubsystem.getInstance().setRobotPosex(turretFieldPos.getX());
    HoodSubsystem.getInstance().setRobotPosex(robotPose.getX());

    updateTargetPose();
    double goalRad =
        calculateTurretgoalRad(robotPose, robotFieldVelocity, chassisSpeeds.omegaRadiansPerSecond);

    if (Math.abs(currentPositionDeg) > UNWIND_THRESHOLD) {
      isUnwinding = true;
      lastUnwindTime = Timer.getFPGATimestamp();
    } else if (isUnwinding && atGoal()) {
      isUnwinding = false;
    } else if (wrappingAround && atGoal()) {
      wrappingAround = false;
    }

    if (isUnwinding) {
      targetAngle = Rotation2d.fromRadians(MathUtil.angleModulus(goalRad));
    } else if (!wrappingAround) {
      targetAngle = findBestTarget(Rotation2d.fromRadians(goalRad), currentPositionDeg);
    }

    targetAngle =
        Rotation2d.fromDegrees(MathUtil.clamp(targetAngle.getDegrees(), MIN_ANGLE, MAX_ANGLE));
    double targetRotations = targetAngle.getDegrees() / 360.0;
    turretMotor.setControl(motionMagicRequest.withPosition(targetRotations));
  }

  private Rotation2d findBestTarget(Rotation2d fieldRelativeTarget, double currentAbsoluteDeg) {
    double wrappedTargetDeg = fieldRelativeTarget.getDegrees();

    double[] candidates = {
      wrappedTargetDeg,
      wrappedTargetDeg + 360,
      wrappedTargetDeg - 360,
      wrappedTargetDeg + 720,
      wrappedTargetDeg - 720
    };

    double bestTarget = currentAbsoluteDeg;
    double smallestError = Double.POSITIVE_INFINITY;

    for (double candidate : candidates) {
      if (candidate < MIN_ANGLE || candidate > MAX_ANGLE) {
        continue;
      }

      double error = Math.abs(candidate - currentAbsoluteDeg);

      if (error < smallestError) {
        smallestError = error;
        bestTarget = candidate;
      }
    }
    if (smallestError > 300) {
      lastTimeAtGoal = Timer.getFPGATimestamp();
      wrappingAround = true;
    }
    return Rotation2d.fromDegrees(bestTarget);
  }

  public double getAbsolutePositionDeg() {
    return turretMotor.getPosition().getValueAsDouble() * 360.0;
  }

  public double getVelocityDegPerSec() {
    return turretMotor.getVelocity().getValueAsDouble() * 360.0;
  }

  public double applyAngularLead(
      double turretAngleRad, double robotOmegaRadPerSec, Pose2d robotPose) {
    double totalTime = ballFlightTimeMap.get(getDistanceFromHub(robotPose));
    return turretAngleRad - robotOmegaRadPerSec * totalTime;
  }

  public double applyTranslationalLead(
      double turretAngleRad,
      Translation2d robotVelocity,
      double robotOmegaRadPerSec,
      Rotation2d robotHeading,
      Translation2d turretToHub) {
    double distance = turretToHub.getNorm();
    if (distance < 0.01) {
      return turretAngleRad;
    }

    Translation2d angularContribution =
        new Translation2d(
                -robotOmegaRadPerSec * robotToTurret.getY(),
                robotOmegaRadPerSec * robotToTurret.getX())
            .rotateBy(robotHeading);
    Translation2d turretVelocity = robotVelocity.plus(angularContribution);

    Translation2d shotDirection = turretToHub.div(distance);

    Translation2d lateralDir = new Translation2d(-shotDirection.getY(), shotDirection.getX());
    double lateralVelocity =
        turretVelocity.getX() * lateralDir.getX() + turretVelocity.getY() * lateralDir.getY();

    double lead = Math.atan2(lateralVelocity * ballFlightTimeMap.get(distance), distance);

    return turretAngleRad - lead;
  }

  public double applyVisionCorrection(
      double turretAngleRad, double txDeg, double turretAngularVelocity) {
    // if moving fast, ignore correction
    if (Math.abs(turretAngularVelocity) > Math.toRadians(120)) {
      return turretAngleRad;
    }
    // vision nudgey
    return turretAngleRad
        + MathUtil.clamp(
            VISION_KP * (Math.toRadians(txDeg) - getAngleFromHub(robotPose).getRadians()),
            -Math.toRadians(5),
            Math.toRadians(5));
  }

  public boolean isVisionTrustworthy(
      double tx, double turretOmega, double robotOmega, double distance, double tid) {
    if (tid == 10 || tid == 25) {
      return Math.abs(tx - getAngleFromHub(robotPose).getDegrees()) < 5.0
          && Math.abs(turretOmega) < Math.toRadians(120)
          && Math.abs(robotOmega) < Math.toRadians(180)
          && distance < 6.0;
    }
    return false; // not hub tag
  }

  private Translation2d getHubPosition() {
    return AllianceFlipUtil.apply(FieldConstants.HUBPOSE.getTranslation());
  }

  public double getDistanceFromHub(Pose2d robotPose) {
    Translation2d turretPose =
        robotPose.getTranslation().plus(robotToTurret.rotateBy(robotPose.getRotation()));

    Translation2d toHub = getHubPosition().minus(turretPose);

    return toHub.getNorm();
  }

  public Rotation2d getAngleFromHub(Pose2d robotPose) {
    Translation2d turretPose =
        robotPose.getTranslation().plus(robotToTurret.rotateBy(robotPose.getRotation()));

    Translation2d toHub = getHubPosition().minus(turretPose);

    return toHub.getAngle();
  }

  public boolean atGoal() {
    double positionError = Math.abs(targetAngle.getDegrees() - getAbsolutePositionDeg());
    if (Timer.getFPGATimestamp() - lastUnwindTime > 2.0) {
      return true; // if we've been unwinding for a while, just say we're at the goal to prevent
      // getting stuck
    }

    if (positionError < 15.0) {
      lastTimeAtGoal = Timer.getFPGATimestamp();
      wrappingAround = false;
      return true;
    }
    return false;
  }

  public boolean isOnTarget() {
    double positionError = Math.abs(targetAngle.getDegrees() - getAbsolutePositionDeg());
    return positionError < 3.0 && !isUnwinding && !wrappingAround && !atSoftLimit;
  }

  // for passing
  public boolean isLooselyOnTarget() {
    double positionError = Math.abs(targetAngle.getDegrees() - getAbsolutePositionDeg());
    return positionError < 15.0 && !isUnwinding && !wrappingAround && !atSoftLimit;
  }

  // pass when near hub (mid Y + mid X) but not in our alliance zone
  public boolean isPassingMode() {
    if (robotPose == null) return false;
    // in our alliance zone — always shoot, never pass
    if (AllianceFlipUtil.shouldFlip()) {
      if (robotPose.getX() > FieldConstants.FIELDLENGTH - FieldConstants.ALLIANCEWALLTOHUB)
        return false;
    } else return !(robotPose.getX() < FieldConstants.ALLIANCEWALLTOHUB);
    return true;
  }

  public boolean justSpinUp() {
    double robotPosex = AllianceFlipUtil.apply(robotPose).getX();
    double midY = FieldConstants.FIELDWIDTH / 2.0;
    double midX = FieldConstants.FIELDLENGTH / 2.0;
    boolean nearMidY = Math.abs(robotPose.getY() - midY) < FieldConstants.FIELDWIDTH * 0.25; // tune
    boolean pastMidX = robotPosex > midX - 2.0; // tune
    if (robotPosex < FieldConstants.ALLIANCEWALLTOHUB) {
      return false;
    } else if (nearMidY && pastMidX) {
      return false;
    } else if (nearMidY && !pastMidX) {
      return true;
    }
    return false;
  }

  public double calculateTurretgoalRad(
      Pose2d robotPose, Translation2d robotVelocity, double robotOmega) {
    // base turret angle robot relative poointing towards hub
    Translation2d turretPose =
        robotPose.getTranslation().plus(robotToTurret.rotateBy(robotPose.getRotation()));
    Translation2d toTargetPose = targetPose.minus(turretPose);
    double fieldAngle = Math.atan2(toTargetPose.getY(), toTargetPose.getX());
    double angle =
        MathUtil.angleModulus(
            fieldAngle - robotPose.getRotation().getRadians() - TURRET_FORWARD_OFFSET_RAD);
    angle = applyAngularLead(angle, robotOmega, robotPose);

    angle =
        applyTranslationalLead(
            angle,
            robotVelocity,
            robotOmega,
            robotPose.getRotation(),
            targetPose.minus(
                robotPose.getTranslation().plus(robotToTurret.rotateBy(robotPose.getRotation()))));

    if (isVisionTrustworthy(
        limelightTable.getEntry("tx").getDouble(0.0),
        Math.toRadians(getVelocityDegPerSec()),
        robotOmega,
        getDistanceFromHub(robotPose),
        limelightTable.getEntry("tid").getDouble(0))) {
      angle =
          applyVisionCorrection(
              angle,
              limelightTable.getEntry("tx").getDouble(0.0),
              Math.toRadians(getVelocityDegPerSec()));
    }

    double clampedAngle =
        MathUtil.clamp(angle, Math.toRadians(MIN_ANGLE), Math.toRadians(MAX_ANGLE));
    atSoftLimit = Math.abs(clampedAngle - angle) > Math.toRadians(1.0);
    return clampedAngle;
  }

  // TODO: maybe make it so it cant shoot in the middle, and switch to the middle of bump when past
  // mid
  public void updateTargetPose() {
    Translation2d hubPos = getHubPosition();
    Translation2d topTarget = AllianceFlipUtil.apply(FieldConstants.TOPTARGET.getTranslation());
    Translation2d bottomTarget =
        AllianceFlipUtil.apply(FieldConstants.BOTTOMTARGET.getTranslation());

    boolean inAllianceZone;
    if (AllianceFlipUtil.shouldFlip()) {
      inAllianceZone =
          robotPose.getX() > FieldConstants.FIELDLENGTH - FieldConstants.ALLIANCEWALLTOHUB;
    } else {
      inAllianceZone = robotPose.getX() < FieldConstants.ALLIANCEWALLTOHUB;
    }

    if (!inAllianceZone) {
      // in mid — target bumps to pass fuel to our side
      if (robotPose.getY() > FieldConstants.FIELDWIDTH / 2) {
        targetPose = topTarget;
      } else {
        targetPose = bottomTarget;
      }
    } else {
      targetPose = hubPos;
    }
  }
}
