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
  private double predictedDistanceToHub = 0.0;

  private static final Translation2d robotToTurret =
      new Translation2d(Units.inchesToMeters(-5.875), 0);
  private final TalonFX turretMotor = new TalonFX(20);
  private final MotionMagicVoltage motionMagicRequest =
      new MotionMagicVoltage(0).withEnableFOC(true);

  private static final double MAX_ANGLE = 80;
  private static final double MIN_ANGLE = -100;
  private static final double TURRET_FORWARD_OFFSET_RAD = Math.PI;
  private static final double MAX_VELOCITY_IN_DEG_PER_SEC = 38279.9988;
  private static final double MAX_ACCELERATION_IN_DEG_PER_SEC = 15000;
  private static final double UNWIND_THRESHOLD = 500;
  private static final double GEAR_RATIO = 50.0;

  private Rotation2d targetAngle;
  private boolean isUnwinding;
  private double lastUnwindTime = Double.POSITIVE_INFINITY;

  private boolean wrappingAround;
  private boolean atSoftLimit = false;
  private boolean visionTrackOverride = false;
  private double turretManualOffsetDeg = 0.0;

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
    ballFlightTimeMap.put(2.5, 0.30 * 2.5); // distance (m) -> flight time (s)
    ballFlightTimeMap.put(2.8, 0.35 * 2.5);
    ballFlightTimeMap.put(3.1, 0.40 * 2.5);
    ballFlightTimeMap.put(3.4, 0.45 * 2.5);
    ballFlightTimeMap.put(3.7, 0.50 * 2.5);
    ballFlightTimeMap.put(4.0, 0.55 * 2.5);
    ballFlightTimeMap.put(4.3, 0.60 * 2.5);
    ballFlightTimeMap.put(4.6, 0.65 * 2.5);
    ballFlightTimeMap.put(5.3, 0.75 * 2.5);
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

    config.Slot0.kP = 67.0; // lower by a lot maybe 60
    config.Slot0.kI = 0.0;
    config.Slot0.kD = 0.0;
    config.Slot0.kS = 0.2;
    config.Slot0.kV = 0.117;
    config.Slot0.kA = 0.0;

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

    // Update shooter and hood with predicted distance (accounts for robot motion)
    double distanceToHub =
        predictedDistanceToHub > 0.01 ? predictedDistanceToHub : getDistanceFromHub(robotPose);
    ShooterSubsystem.getInstance().setTargetDistance(distanceToHub);
    HoodSubsystem.getInstance().setTargetDistance(distanceToHub);
    //    System.out.println(Units.metersToInches(distanceToHub));
    Translation2d turretFieldPos =
        robotPose.getTranslation().plus(robotToTurret.rotateBy(robotPose.getRotation()));
    //    HoodSubsystem.getInstance().setRobotPosex(turretFieldPos.getX());
    HoodSubsystem.getInstance().setRobotPosex(robotPose.getX());

    updateTargetPose();

    // ── VISION TRACK OVERRIDE ──────────────────────────────────────────────
    // Turret follows TX from limelight-under (fixed on robot chassis, not turret).
    // TX = angle from camera to target. Convert to turret-relative angle.
    // Adjust sign/offset below to match your camera mounting direction.
    if (visionTrackOverride) {
      boolean hasTarget = LimelightHelpers.getTV("limelight-under");
      if (hasTarget) {
        double tx = LimelightHelpers.getTX("limelight-under");
        double turretAngleDeg = -tx;
        targetAngle = Rotation2d.fromDegrees(MathUtil.clamp(turretAngleDeg, MIN_ANGLE, MAX_ANGLE));
      }
      // no target: hold last targetAngle
      double targetRotations = targetAngle.getDegrees() / 360.0;
      turretMotor.setControl(motionMagicRequest.withPosition(targetRotations));
      return;
    }
    // ── END VISION TRACK OVERRIDE ──────────────────────────────────────────

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

    double offsetDeg = targetAngle.getDegrees() + turretManualOffsetDeg;
    targetAngle = Rotation2d.fromDegrees(MathUtil.clamp(offsetDeg, MIN_ANGLE, MAX_ANGLE));
    double targetRotations = targetAngle.getDegrees() / 360.0;
    turretMotor.setControl(
        motionMagicRequest.withPosition(targetRotations)); // change back for turret to work
    turretMotor.setControl(
        motionMagicRequest.withPosition(targetRotations)); // change back for turret to work

    org.littletonrobotics.junction.Logger.recordOutput(
        "Turret/ManualOffsetDeg", turretManualOffsetDeg);
    org.littletonrobotics.junction.Logger.recordOutput("Turret/CanShoot", canShoot());
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

  public void setVisionTrackOverride(boolean enable) {
    visionTrackOverride = enable;
  }

  public boolean getVisionTrackOverride() {
    return visionTrackOverride;
  }

  public void setTurretManualOffset(double deg) {
    turretManualOffsetDeg = MathUtil.clamp(deg, -17.0, 17.0);
  }

  public void resetTurretManualOffset() {
    turretManualOffsetDeg = 0.0;
  }

  public double getTurretManualOffset() {
    return turretManualOffsetDeg;
  }

  /** Returns true when the robot is in the alliance zone and not inside the tower/ladder. */
  public boolean canShoot() {
    if (robotPose == null) return false;
    if (FieldConstants.isInsideTower(robotPose)) return false;

    if (AllianceFlipUtil.shouldFlip()) {
      return robotPose.getX() > FieldConstants.FIELDLENGTH - FieldConstants.ALLIANCEWALLTOHUB;
    } else {
      return robotPose.getX() < FieldConstants.ALLIANCEWALLTOHUB;
    }
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
    boolean nearMidY = Math.abs(robotPose.getY() - midY) < FieldConstants.FIELDWIDTH * 0.10; // tune
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

  // ── LEAD MODE SWITCH ────────────────────────────────────────────────────
  // To swap: comment out one calculateTurretgoalRad, uncomment the other.

  // ── OPTION A: Original angular + translational lead — ACTIVE ──
  //  public double calculateTurretgoalRad(
  //      Pose2d robotPose, Translation2d robotVelocity, double robotOmega) {
  //    // base turret angle robot relative poointing towards hub
  //    Translation2d turretPose =
  //        robotPose.getTranslation().plus(robotToTurret.rotateBy(robotPose.getRotation()));
  //    Translation2d toTargetPose = targetPose.minus(turretPose);
  //    double fieldAngle = Math.atan2(toTargetPose.getY(), toTargetPose.getX());
  //    double angle =
  //        MathUtil.angleModulus(
  //            fieldAngle - robotPose.getRotation().getRadians() - TURRET_FORWARD_OFFSET_RAD);
  //    angle = applyAngularLead(angle, robotOmega, robotPose);
  //
  //    angle =
  //        applyTranslationalLead(
  //            angle,
  //            robotVelocity,
  //            robotOmega,
  //            robotPose.getRotation(),
  //            targetPose.minus(
  //
  // robotPose.getTranslation().plus(robotToTurret.rotateBy(robotPose.getRotation()))));
  //
  //    if (isVisionTrustworthy(
  //        limelightTable.getEntry("tx").getDouble(0.0),
  //        Math.toRadians(getVelocityDegPerSec()),
  //        robotOmega,
  //        getDistanceFromHub(robotPose),
  //        limelightTable.getEntry("tid").getDouble(0))) {
  //      angle =
  //          applyVisionCorrection(
  //              angle,
  //              limelightTable.getEntry("tx").getDouble(0.0),
  //              Math.toRadians(getVelocityDegPerSec()));
  //    }
  //
  //    double clampedAngle =
  //        MathUtil.clamp(angle, Math.toRadians(MIN_ANGLE), Math.toRadians(MAX_ANGLE));
  //    atSoftLimit = Math.abs(clampedAngle - angle) > Math.toRadians(1.0);
  //    return clampedAngle;
  //  }

  // ── OPTION B: Iterative convergence lead — INACTIVE ──
  private static final int LEAD_ITERATIONS = 3;

  public double calculateTurretgoalRad(
      Pose2d robotPose, Translation2d robotVelocity, double robotOmega) {
    // predict where robot will be when ball arrives, aim from there, repeat
    double t = 0.0;

    for (int i = 0; i < LEAD_ITERATIONS; i++) {
      double futureX = robotPose.getX() + robotVelocity.getX() * t;
      double futureY = robotPose.getY() + robotVelocity.getY() * t;
      double futureHeading = robotPose.getRotation().getRadians() + robotOmega * t;
      Rotation2d futureRot = Rotation2d.fromRadians(futureHeading);

      Translation2d futureTurretPose =
          new Translation2d(futureX, futureY).plus(robotToTurret.rotateBy(futureRot));
      double distance = targetPose.minus(futureTurretPose).getNorm();
      t = ballFlightTimeMap.get(Math.max(distance, 2.5));
    }

    double futureX = robotPose.getX() + robotVelocity.getX() * t;
    double futureY = robotPose.getY() + robotVelocity.getY() * t;
    double futureHeading = robotPose.getRotation().getRadians() + robotOmega * t;
    Rotation2d futureRot = Rotation2d.fromRadians(futureHeading);

    Translation2d futureTurretPose =
        new Translation2d(futureX, futureY).plus(robotToTurret.rotateBy(futureRot));
    Translation2d toTarget = targetPose.minus(futureTurretPose);
    predictedDistanceToHub = toTarget.getNorm();
    double fieldAngle = Math.atan2(toTarget.getY(), toTarget.getX());

    // robot-relative using CURRENT heading (turret is on the current robot)
    double angle =
        MathUtil.angleModulus(
            fieldAngle - robotPose.getRotation().getRadians() - TURRET_FORWARD_OFFSET_RAD);

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

  // ── END LEAD MODE SWITCH ──────────────────────────────────────────────

  public void updateTargetPose() {
    Translation2d hubPos = getHubPosition();

    // Our wall and opponent wall X in blue-origin coords
    double ourWallX =
        AllianceFlipUtil.shouldFlip()
            ? FieldConstants.FIELDLENGTH - FieldConstants.ALLIANCEWALLTOHUB
            : FieldConstants.ALLIANCEWALLTOHUB;
    double opponentBumpX =
        AllianceFlipUtil.shouldFlip()
            ? FieldConstants.ALLIANCEWALLTOHUB
            : FieldConstants.FIELDLENGTH - FieldConstants.ALLIANCEWALLTOHUB;

    // Our bumps (pass toward our alliance wall) vs opponent bumps (pass away from hub)
    Translation2d topOurBump = AllianceFlipUtil.apply(FieldConstants.TOPBUMPMID.getTranslation());
    Translation2d bottomOurBump =
        AllianceFlipUtil.apply(FieldConstants.BOTTOMBUMPMID.getTranslation());
    Translation2d topOpponentBump =
        new Translation2d(opponentBumpX, AllianceFlipUtil.applyY(FieldConstants.TOPBUMPMID.getY()));
    Translation2d bottomOpponentBump =
        new Translation2d(
            opponentBumpX, AllianceFlipUtil.applyY(FieldConstants.BOTTOMBUMPMID.getY()));

    // Three zones: alliance, middle, opponent's side
    // Alliance zone boundary: our hub
    // Opponent zone boundary: opponent's hub (mirrored)
    double ourHubX =
        AllianceFlipUtil.shouldFlip()
            ? FieldConstants.FIELDLENGTH - FieldConstants.ALLIANCEWALLTOHUB
            : FieldConstants.ALLIANCEWALLTOHUB;
    double opponentHubX =
        AllianceFlipUtil.shouldFlip()
            ? FieldConstants.ALLIANCEWALLTOHUB
            : FieldConstants.FIELDLENGTH - FieldConstants.ALLIANCEWALLTOHUB;

    boolean inAllianceZone;
    boolean onOpponentSide;
    if (AllianceFlipUtil.shouldFlip()) {
      inAllianceZone = robotPose.getX() > ourHubX;
      onOpponentSide = robotPose.getX() < opponentHubX;
    } else {
      inAllianceZone = robotPose.getX() < ourHubX;
      onOpponentSide = robotPose.getX() > opponentHubX;
    }

    boolean topHalf =
        AllianceFlipUtil.shouldFlip()
            ? robotPose.getY() < FieldConstants.FIELDWIDTH / 2
            : robotPose.getY() > FieldConstants.FIELDWIDTH / 2;

    if (inAllianceZone) {
      targetPose = hubPos;
    } else if (onOpponentSide) {
      // Hub blocks line-of-sight to our bumps — aim at opponent's bumps
      targetPose = topHalf ? topOpponentBump : bottomOpponentBump;
    } else {
      // Middle — aim at outpost if on outpost side, otherwise our bump
      Translation2d outpostPos =
          AllianceFlipUtil.apply(FieldConstants.OUTPOSTPOSE.getTranslation());
      boolean outpostIsTopHalf =
          AllianceFlipUtil.shouldFlip()
              ? outpostPos.getY() < FieldConstants.FIELDWIDTH / 2
              : outpostPos.getY() > FieldConstants.FIELDWIDTH / 2;
      if (topHalf == outpostIsTopHalf) {
        targetPose = outpostPos;
      } else {
        targetPose = topHalf ? topOurBump : bottomOurBump;
      }
    }
  }
}
