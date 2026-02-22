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

public class turrettestingSubsystem extends SubsystemBase {

  //  private static final turrettestingSubsystem INSTANCE = new turrettestingSubsystem();
  //
  //  public static turrettestingSubsystem getInstance() {
  //    return INSTANCE;
  //  }

  private static double VISION_LATENCY;

  private final NetworkTable limelightTable;
  private final Translation2d HUB_POSE =
      AllianceFlipUtil.apply(FieldConstants.HUBPOSE.getTranslation());
  private final Translation2d TOPTARGET =
      AllianceFlipUtil.apply(FieldConstants.TOPTARGET.getTranslation());
  private final Translation2d BOTTOMTARGET =
      AllianceFlipUtil.apply(FieldConstants.BOTTOMTARGET.getTranslation());
  private Translation2d TARGET_POSE = HUB_POSE;
  private static final double VISION_KP = 0.015;
  Pose2d robotPose;
  private final Supplier<Pose2d> poseSupplier;
  private final Supplier<ChassisSpeeds> chassisSpeedsSupplier;

  private Translation2d robotFieldVelocity;

  private static final Translation2d robotToTurret = new Translation2d(0.35, 0.10);

  private final TalonFX turretMotor = new TalonFX(20);
  private final MotionMagicVoltage motionMagicRequest = new MotionMagicVoltage(0);

  private static final double MAX_ANGLE = 720;
  private static final double MIN_ANGLE = -720;
  private static final double MAX_VELOCITY_IN_DEG_PER_SEC = 35000;
  private static final double MAX_ACCELERATION_IN_DEG_PER_SEC = 1000;
  private static final double UNWIND_THRESHOLD = 500;
  private static final double GEAR_RATIO = 60.8;

  private Rotation2d targetAngle;
  private boolean isUnwinding;
  public double lastUnwindTime = 0.0;

  private boolean wrappingAround;

  private double lastTimeAtGoal = 0;

  private static final InterpolatingDoubleTreeMap ballFlightTimeMap =
      new InterpolatingDoubleTreeMap();

  static {
    // this is example later replace
    ballFlightTimeMap.put(2.0, 0.42); // distance in meters, time in seconds
  }

  public turrettestingSubsystem(
      Supplier<Pose2d> poseSupplier, Supplier<ChassisSpeeds> chassisSpeedsSupplier) {
    this.poseSupplier = poseSupplier;
    this.chassisSpeedsSupplier = chassisSpeedsSupplier;
    limelightTable = NetworkTableInstance.getDefault().getTable("limelight-three");
    if (AllianceFlipUtil.shouldFlip()) {
      limelightTable.getEntry("priorityid").setNumber(25); // focus on blue hub tag
    } else {
      limelightTable.getEntry("priorityid").setNumber(10); // focus on red hub tag
      System.out.print(
          limelightTable
              .getEntry("priorityid")
              .setNumber(10)); // see if it prints out false to debug
    }

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

    config.Slot0.kP = 75.4; // lower by a lot maybe
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
    turretMotor.setPosition(0);
  }

  @Override
  public void periodic() {
    LimelightHelpers.setCameraPose_RobotSpace(
        "limelight",
        0.0127, // x meters forward
        -0.1143, // y meters left
        0, // z meters up
        0, // roll deg
        0, // pitch deg
        getAbsolutePositionDeg() + 90 // yaw
        ); // yaw deg
    // This method will be called once per scheduler run
    VISION_LATENCY =
        (limelightTable.getEntry("tl").getDouble(0.0)
                + limelightTable.getEntry("cl").getDouble(0.0))
            / 1000.0;
    robotPose = poseSupplier.get();
    robotFieldVelocity =
        new Translation2d(
                chassisSpeedsSupplier.get().vxMetersPerSecond,
                chassisSpeedsSupplier.get().vyMetersPerSecond)
            .rotateBy(robotPose.getRotation());

    double currentPositionDeg = getAbsolutePositionDeg();

    setTARGET_POSE();
    if (Math.abs(currentPositionDeg) > UNWIND_THRESHOLD) {

      isUnwinding = true;
      //      targetAngle = UNWIND_TARGET;
      targetAngle =
          Rotation2d.fromRadians(
              MathUtil.angleModulus(
                  calculateTurretgoalRad(
                      robotPose,
                      robotFieldVelocity,
                      chassisSpeedsSupplier.get().omegaRadiansPerSecond)));
      lastUnwindTime = Timer.getFPGATimestamp();
      //      System.out.println("unwinding");
    } else if (isUnwinding && atGoal()) {
      // Finished unwinding
      //      System.out.println(
      //
      // "DONEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEE");
      isUnwinding = false;
    } else if (wrappingAround && atGoal()) {
      wrappingAround = false;
    }
    if (!isUnwinding && !wrappingAround) {
      targetAngle =
          findBestTarget(
              Rotation2d.fromRadians(
                  calculateTurretgoalRad(
                      robotPose,
                      robotFieldVelocity,
                      chassisSpeedsSupplier.get().omegaRadiansPerSecond)),
              currentPositionDeg);
      //      System.out.println("targeting");
    }
    if (isUnwinding) {
      System.out.println("is unwinding");
      //      targetAngle = UNWIND_TARGET;
      targetAngle =
          Rotation2d.fromRadians(
              MathUtil.angleModulus(
                  calculateTurretgoalRad(
                      robotPose,
                      robotFieldVelocity,
                      chassisSpeedsSupplier.get().omegaRadiansPerSecond)));
    }

    targetAngle =
        Rotation2d.fromDegrees(MathUtil.clamp(targetAngle.getDegrees(), MIN_ANGLE, MAX_ANGLE));
    double targetRotations = targetAngle.getDegrees() / 360.0;
    turretMotor.setControl(motionMagicRequest.withPosition(targetRotations));
  }

  public void setTarget(Pose2d robotPose, Translation2d robotVelocity, double robotOmegaRadPerSec) {
    double tx = limelightTable.getEntry("tx").getDouble(0.0);
    double tv = limelightTable.getEntry("tv").getDouble(0.0);

    //    if (tv < 1) {
    //      //      System.out.print("no target");
    //
    //      //      turretMotor.setControl(voltageRequest.withOutput(0.0));
    //      //      targetAngle = Rotation2d.fromDegrees(getAbsolutePositionDeg());
    //      targetAngle = targetAngle; // hold position
    //      return;
    //    }
    Translation2d turretPose =
        robotPose.getTranslation().plus(robotToTurret.rotateBy(robotPose.getRotation()));

    Translation2d toTargetPose = TARGET_POSE.minus(turretPose);
    // field relative angle
    double fieldAngle = Math.atan2(toTargetPose.getY(), toTargetPose.getX());

    double desiredTurretAngle =
        MathUtil.angleModulus(fieldAngle - robotPose.getRotation().getRadians());

    //    // see if we need to unwind
    //    if (Math.abs(turretMotor.getPosition().getValueAsDouble()) > UNWIND_THRESHOLD) {
    //      // need to unwind
    //      isUnwinding = true;
    //      targetAngle = UNWIND_TARGET;
    //    } else {
    // it ok no need to unwind
    //    isUnwinding = false;
    //    System.out.print("has target");
    targetAngle = Rotation2d.fromRadians(desiredTurretAngle);

    //            findBestTarget(
    //            Rotation2d.fromRadians(desiredTurretAngle),
    //            getAbsolutePositionDeg()
    //            );

    //        findBestTarget(
    //            Rotation2d.fromDegrees(tx + getAbsolutePositionDeg()),
    //            getAbsolutePositionDeg()); // target from vision

    //    }
  }

  private Rotation2d findBestTarget(Rotation2d fieldRelativeTarget, double currentAbsoluteDeg) {
    double wrappedTargetDeg = fieldRelativeTarget.getDegrees();

    double[] canididates = {
      wrappedTargetDeg,
      wrappedTargetDeg + 360,
      wrappedTargetDeg - 360,
      wrappedTargetDeg + 720,
      wrappedTargetDeg - 720
    };

    double bestTarget = currentAbsoluteDeg;
    double smallestError = Double.POSITIVE_INFINITY;

    for (double candidate : canididates) {
      if (candidate < MIN_ANGLE || candidate > MAX_ANGLE) {
        continue;
      }

      double error = Math.abs(candidate - currentAbsoluteDeg);

      if (error < smallestError) {
        smallestError = error;
        bestTarget = candidate;
      }
    }
    //    System.out.println(smallestError);   change back
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
    double totalTime;
    if (isVisionTrustworthy( // if priorityid works dont need this
        limelightTable.getEntry("tx").getDouble(0.0),
        Math.toRadians(getVelocityDegPerSec()),
        robotOmegaRadPerSec,
        getDistanceFromHub(robotPose),
        limelightTable.getEntry("tid").getDouble(0))) {
      totalTime = VISION_LATENCY + ballFlightTimeMap.get(getDistanceFromHub(robotPose));
    } else {
      totalTime = ballFlightTimeMap.get(getDistanceFromHub(robotPose));
    }
    return turretAngleRad - robotOmegaRadPerSec * totalTime;
  }

  public double applyTranslationalLead(
      double turretAngleRad,
      Translation2d robotVelocity,
      double robotOmegaRadPerSec,
      Rotation2d robotHeading,
      Translation2d turretToHub) {
    Translation2d turretVelocity =
        robotVelocity.plus(
            new Translation2d(
                -robotOmegaRadPerSec * robotToTurret.getY(),
                robotOmegaRadPerSec * robotToTurret.getX()));

    Translation2d shotDirection = turretToHub.div(turretToHub.getNorm());

    Translation2d lateralDir = new Translation2d(-shotDirection.getY(), shotDirection.getX());

    double distance = turretToHub.getNorm();
    double lateralVelocity =
        turretVelocity.getX() * lateralDir.getX() + turretVelocity.getY() * lateralDir.getY();

    double lead = Math.atan2(lateralVelocity * ballFlightTimeMap.get(distance), distance);

    return turretAngleRad + lead;
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

  public double getDistanceFromHub(Pose2d robotPose) {
    Translation2d turretPose =
        robotPose.getTranslation().plus(robotToTurret.rotateBy(robotPose.getRotation()));

    Translation2d toHub = HUB_POSE.minus(turretPose);

    return toHub.getNorm();
  }

  public Rotation2d getAngleFromHub(Pose2d robotPose) {
    Translation2d turretPose =
        robotPose.getTranslation().plus(robotToTurret.rotateBy(robotPose.getRotation()));

    Translation2d toHub = HUB_POSE.minus(turretPose);

    return toHub.getAngle();
  }

  public boolean atGoal() {
    double positionError = Math.abs(targetAngle.getDegrees() - getAbsolutePositionDeg());
    double velocityError = Math.abs(getVelocityDegPerSec());
    System.out.println(positionError);
    //    System.out.println(positionError + "   egrnkjnkjrekjgnkj   " + velocityError);
    if (Timer.getFPGATimestamp() - lastUnwindTime > 2.0) {
      return true; // if we've been unwinding for a while, just say we're at the goal to prevent
      // getting stuck
    }

    if (positionError < 8.0) {
      lastTimeAtGoal = Timer.getFPGATimestamp();
      wrappingAround = false;
      return true;
    }
    //    return positionError < 25.0 && velocityError < 3.0;
    return false;
  }

  public boolean isOnTarget() {
    double positionError = Math.abs(targetAngle.getDegrees() - getAbsolutePositionDeg());
    return positionError < 3.0 && !isUnwinding && !wrappingAround;
  }

// for passing
  public boolean isLooselyOnTarget() {
    double positionError = Math.abs(targetAngle.getDegrees() - getAbsolutePositionDeg());
    return positionError < 15.0 && !isUnwinding;
  }

  //see if pass mid, if so then pass
  public boolean isPassingMode() {
    if (robotPose == null) return false;
    if (AllianceFlipUtil.shouldFlip()) {
      return robotPose.getX() < FieldConstants.FIELDLENGTH - FieldConstants.ALLIANCEWALLTOHUB;
    } else {
      return robotPose.getX() > FieldConstants.ALLIANCEWALLTOHUB;
    }
  }

  public double calculateTurretgoalRad(
      Pose2d robotPose, Translation2d robotVelocity, double robotOmega) {
    setTarget(robotPose, robotVelocity, robotOmega);
    double angle = targetAngle.getRadians();
    angle = applyAngularLead(angle, robotOmega, robotPose);

    angle =
        applyTranslationalLead(
            angle,
            robotVelocity,
            robotOmega,
            robotPose.getRotation(),
            TARGET_POSE.minus(
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

    angle = MathUtil.clamp(angle, Math.toRadians(MIN_ANGLE), Math.toRadians(MAX_ANGLE));
    return angle;
  }
  // TODO: maybe make it so it cant shoot in the middle, and switch to the middle of bump when past
  // mid
  public void setTARGET_POSE() {
    if (AllianceFlipUtil.shouldFlip()) { // on red alliance

      if (robotPose.getX()
          < FieldConstants.FIELDLENGTH
              - FieldConstants
                  .ALLIANCEWALLTOHUB) { // if its in mid target the side walls to put balls over to
        // our side
        if (robotPose.getY() > FieldConstants.FIELDWIDTH / 2) {
          TARGET_POSE =
              TOPTARGET; // top half of field, target top right bump to get fuel to our side
        } else { // maybe do something here to tell feeder we changing, so we dont fire when we
          // arent at goal, or use atGoal()
          TARGET_POSE =
              BOTTOMTARGET; // bottom half of field, target bottom left bump to get fuel to our
          // side
        }
      } else {
        TARGET_POSE = HUB_POSE; // if not target our hub
      }

    } else { // blue alliance

      if (robotPose.getX()
          > FieldConstants
              .ALLIANCEWALLTOHUB) { // if its in mid target the side walls to put balls over to our
        // side
        if (robotPose.getY() > FieldConstants.FIELDWIDTH / 2) {
          TARGET_POSE =
              TOPTARGET; // top half of field, target top right bump to get fuel to our side
        } else { // maybe do something here to tell feeder we changing, so we dont fire when we
          // arent at goal, or use atGoal()
          TARGET_POSE =
              BOTTOMTARGET; // bottom half of field, target bottom left bump to get fuel to our
          // side
        }
      } else {
        TARGET_POSE = HUB_POSE; // if not target our hub
      }
    }
  }
}
