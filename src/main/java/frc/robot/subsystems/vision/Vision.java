// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.vision;

import static frc.robot.subsystems.vision.VisionConstants.*;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.vision.VisionIO.PoseObservationType;
import java.util.LinkedList;
import java.util.List;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/**
 * Subsystem that processes AprilTag observations from multiple cameras and feeds filtered, weighted
 * pose estimates into the drive subsystem's SwerveDrivePoseEstimator.
 *
 * <p>DATA FLOW:
 *
 * <ol>
 *   <li>Each VisionIO (one per camera) produces raw PoseObservations via NetworkTables
 *   <li>This class filters them (rejection checks) and computes per-observation standard deviations
 *   <li>Accepted observations are passed to the VisionConsumer (Drive::addVisionMeasurement)
 *   <li>The SwerveDrivePoseEstimator fuses them with odometry using a Kalman filter
 * </ol>
 *
 * <p>ANTI-OSCILLATION: When multiple cameras disagree on the robot's pose, the estimator can
 * ping-pong between them. Three defenses prevent this:
 *
 * <ol>
 *   <li>Pose sanity check — reject observations > maxPoseDisagreement from current estimate
 *   <li>Gyro rate rejection — reject all vision above maxGyroRateForVision (blurry frames)
 *   <li>Camera trust differentiation — LL4 gets ~3x more weight than LL3 via cameraStdDevFactors
 * </ol>
 *
 * <p>See VisionConstants.java for all tuning values, reference values from top teams, and a tuning
 * guide.
 */
public class Vision extends SubsystemBase {
  private final VisionConsumer consumer;
  private final Supplier<Pose2d> poseSupplier; // Current estimated pose, for sanity checking
  private final VisionIO[] io;
  private final VisionIOInputsAutoLogged[] inputs;
  private final Alert[] disconnectedAlerts;
  private final DoubleSupplier gyroRateSupplier; // Robot angular velocity in rad/s

  /** Convenience constructor with no pose or gyro rate info (for testing/replay). */
  public Vision(VisionConsumer consumer, VisionIO... io) {
    this(consumer, Pose2d::new, () -> 0.0, io);
  }

  /**
   * Full constructor. In RobotContainer, typically called as:
   *
   * <pre>
   * new Vision(drive::addVisionMeasurement, drive::getPose,
   *            () -> drive.getChassisSpeeds().omegaRadiansPerSecond, visionIO1, visionIO2, ...)
   * </pre>
   *
   * @param consumer Callback that feeds accepted poses into the pose estimator
   * @param poseSupplier Current estimated robot pose, used for the pose sanity check
   * @param gyroRateSupplier Current robot angular velocity in rad/s, used for gyro rate rejection
   *     and stddev scaling
   * @param io One VisionIO per camera. Array index determines which cameraStdDevFactors entry is
   *     used.
   */
  public Vision(
      VisionConsumer consumer,
      Supplier<Pose2d> poseSupplier,
      DoubleSupplier gyroRateSupplier,
      VisionIO... io) {
    this.consumer = consumer;
    this.poseSupplier = poseSupplier;
    this.gyroRateSupplier = gyroRateSupplier;
    this.io = io;

    // Initialize inputs
    this.inputs = new VisionIOInputsAutoLogged[io.length];
    for (int i = 0; i < inputs.length; i++) {
      inputs[i] = new VisionIOInputsAutoLogged();
    }

    // Initialize disconnected alerts
    this.disconnectedAlerts = new Alert[io.length];
    for (int i = 0; i < inputs.length; i++) {
      disconnectedAlerts[i] =
          new Alert(
              "Vision camera " + Integer.toString(i) + " is disconnected.", AlertType.kWarning);
    }
  }

  /**
   * Returns the X angle to the best target, which can be used for simple servoing with vision.
   *
   * @param cameraIndex The index of the camera to use.
   */
  public Rotation2d getTargetX(int cameraIndex) {
    return inputs[cameraIndex].latestTargetObservation.tx();
  }

  @Override
  public void periodic() {
    for (int i = 0; i < io.length; i++) {
      io[i].updateInputs(inputs[i]);
      Logger.processInputs("Vision/Cameralimelight" + Integer.toString(i), inputs[i]);
    }

    // Initialize logging values
    List<Pose3d> allTagPoses = new LinkedList<>();
    List<Pose3d> allRobotPoses = new LinkedList<>();
    List<Pose3d> allRobotPosesAccepted = new LinkedList<>();
    List<Pose3d> allRobotPosesRejected = new LinkedList<>();

    // Loop over cameras
    for (int cameraIndex = 0; cameraIndex < io.length; cameraIndex++) {
      // Update disconnected alert
      disconnectedAlerts[cameraIndex].set(!inputs[cameraIndex].connected);

      // Initialize logging values
      List<Pose3d> tagPoses = new LinkedList<>();
      List<Pose3d> robotPoses = new LinkedList<>();
      List<Pose3d> robotPosesAccepted = new LinkedList<>();
      List<Pose3d> robotPosesRejected = new LinkedList<>();

      // Add tag poses
      for (int tagId : inputs[cameraIndex].tagIds) {
        var tagPose = aprilTagLayout.getTagPose(tagId);
        if (tagPose.isPresent()) {
          tagPoses.add(tagPose.get());
        }
      }

      // Loop over pose observations from this camera.
      // Each VisionIOLimelight produces one MT2 observation per frame (MT1 is skipped).
      for (var observation : inputs[cameraIndex].poseObservations) {
        // LL4 (global shutter) cameras are identified by having a cameraStdDevFactor < 1.0.
        // They get a longer single-tag range because global shutter = less distortion at distance.
        double singleTagLimit =
            (cameraIndex < cameraStdDevFactors.length && cameraStdDevFactors[cameraIndex] < 1.0)
                ? maxSingleTagDistanceLL4
                : maxSingleTagDistance;

        // --- REJECTION CHECKS ---
        // Any observation that fails these is discarded entirely (not fed to the estimator).
        // Rejected observations are still logged to Vision/Camera*/RobotPosesRejected for
        // debugging in AdvantageScope. If too many observations are being rejected, check
        // these thresholds in VisionConstants.java.
        boolean rejectPose =
            // Basic validity: must see at least one tag
            observation.tagCount() == 0
                // Single-tag PnP can produce two valid solutions (ambiguity). High ambiguity
                // means the solver can't tell which is correct. Multi-tag doesn't have this
                // problem because multiple tags constrain the solution uniquely.
                || (observation.tagCount() == 1 && observation.ambiguity() > maxAmbiguity)
                // Single-tag accuracy degrades with distance. Multi-tag is fine farther out.
                || (observation.tagCount() == 1
                    && observation.averageTagDistance() > singleTagLimit)
                // Z should be near 0 for a robot on a flat field. Large Z = bad solve.
                || Math.abs(observation.pose().getZ()) > maxZError
                // Pose must be within the field boundaries (with no margin — could add 0.5m)
                || observation.pose().getX() < 0.0
                || observation.pose().getX() > aprilTagLayout.getFieldLength()
                || observation.pose().getY() < 0.0
                || observation.pose().getY() > aprilTagLayout.getFieldWidth()
                // GYRO RATE REJECTION: when the robot is spinning fast, rolling shutter cameras
                // (LL3) produce smeared frames that give wildly wrong poses. LL4 global shutter
                // is better but still degrades. 3847 Spectrum uses 1.6 rad/s as their threshold.
                // Below this threshold, stddevs are still scaled up by omega (see below).
                || Math.abs(gyroRateSupplier.getAsDouble()) > maxGyroRateForVision
                // POSE SANITY CHECK: reject observations that disagree too much with the current
                // estimated pose. This is the primary defense against multi-camera oscillation.
                // Without this, if camera A says X=5.0 and camera B says X=5.1, the estimator
                // ping-pongs between them every cycle. With this, once the estimator settles,
                // outlier observations get rejected instead of causing oscillation.
                // 254 uses yaw-diff + tag-area checks; 3847 uses 0.25-0.5m; 1678 uses 2.0m.
                || observation
                        .pose()
                        .toPose2d()
                        .getTranslation()
                        .getDistance(poseSupplier.get().getTranslation())
                    > maxPoseDisagreement;

        // Add pose to log
        robotPoses.add(observation.pose());
        if (rejectPose) {
          robotPosesRejected.add(observation.pose());
        } else {
          robotPosesAccepted.add(observation.pose());
        }

        // Skip if rejected
        if (rejectPose) {
          continue;
        }

        // --- STANDARD DEVIATION CALCULATION ---
        // The std dev tells the Kalman filter how much to trust this observation.
        // Higher stddev = less trust = estimator changes less. Lower = more trust = faster update.
        //
        // Formula: baseline * (distance^exponent / tagCount^2) * megatag2Factor * cameraFactor
        //          * omegaFactor
        //
        // Why tagCount^2 (not linear): multi-tag observations are geometrically much more
        // constrained than single-tag. The squared divisor reflects this — 2 tags = 4x more
        // trusted, 3 tags = 9x. 6328 uses the same tagCount^2 scaling.
        double distFactor = Math.pow(observation.averageTagDistance(), distanceExponent);
        double tagFactor = Math.pow(observation.tagCount(), 2.0);
        double stdDevFactor = distFactor / tagFactor;
        double linearStdDev = linearStdDevBaseline * stdDevFactor;
        double angularStdDev = angularStdDevBaseline * stdDevFactor;

        // MT2 (gyro-constrained) observations get tighter linear stddevs because the rotation
        // is fixed from the gyro, making the translation solve more stable. The angular
        // component is set to INFINITY because MT2's rotation IS the gyro input — trusting
        // it would be circular.
        if (observation.type() == PoseObservationType.MEGATAG_2) {
          linearStdDev *= linearStdDevMegatag2Factor;
          angularStdDev *= angularStdDevMegatag2Factor;
        }

        // Per-camera trust weighting. LL4 (global shutter, factor=0.5) gets ~3x more weight
        // than LL3 (rolling shutter, factor=1.5). This is the main mechanism that resolves
        // multi-camera disagreement — the estimator preferentially follows the better camera.
        if (cameraIndex < cameraStdDevFactors.length) {
          linearStdDev *= cameraStdDevFactors[cameraIndex];
          angularStdDev *= cameraStdDevFactors[cameraIndex];
        }

        // SINGLE-TAG ANGULAR INFINITY: single-tag PnP rotation estimates are unreliable,
        // especially at distance or with rolling shutter. Setting angular stddev to infinity
        // means "only use this observation for XY translation, not rotation." The gyro handles
        // rotation much more accurately. Every top team does this (6328, 1678, 3847).
        if (observation.tagCount() == 1) {
          angularStdDev = Double.POSITIVE_INFINITY;
        }

        // OMEGA SCALING: even below the hard rejection threshold, camera frames degrade
        // with rotation speed. This linearly scales stddevs up with omega, making the
        // estimator trust vision less during turns without rejecting entirely.
        double omegaFactor = 1.0 + 2.0 * Math.abs(gyroRateSupplier.getAsDouble());
        linearStdDev *= omegaFactor;
        angularStdDev *= omegaFactor;

        // Feed the accepted, weighted observation into the SwerveDrivePoseEstimator.
        // The estimator uses the stddevs to compute Kalman gains — low stddev = big correction,
        // high stddev = small correction, infinity = ignored for that dimension.
        consumer.accept(
            observation.pose().toPose2d(),
            observation.timestamp(),
            VecBuilder.fill(linearStdDev, linearStdDev, angularStdDev));
      }

      // Log camera metadata
      Logger.recordOutput(
          "Vision/Camera" + Integer.toString(cameraIndex) + "/TagPoses",
          tagPoses.toArray(new Pose3d[0]));
      Logger.recordOutput(
          "Vision/Camera" + Integer.toString(cameraIndex) + "/RobotPoses",
          robotPoses.toArray(new Pose3d[0]));
      Logger.recordOutput(
          "Vision/Camera" + Integer.toString(cameraIndex) + "/RobotPosesAccepted",
          robotPosesAccepted.toArray(new Pose3d[0]));
      Logger.recordOutput(
          "Vision/Camera" + Integer.toString(cameraIndex) + "/RobotPosesRejected",
          robotPosesRejected.toArray(new Pose3d[0]));
      allTagPoses.addAll(tagPoses);
      allRobotPoses.addAll(robotPoses);
      allRobotPosesAccepted.addAll(robotPosesAccepted);
      allRobotPosesRejected.addAll(robotPosesRejected);
    }

    // Log summary data
    Logger.recordOutput("Vision/Summary/TagPoses", allTagPoses.toArray(new Pose3d[0]));
    Logger.recordOutput("Vision/Summary/RobotPoses", allRobotPoses.toArray(new Pose3d[0]));
    Logger.recordOutput(
        "Vision/Summary/RobotPosesAccepted", allRobotPosesAccepted.toArray(new Pose3d[0]));
    Logger.recordOutput(
        "Vision/Summary/RobotPosesRejected", allRobotPosesRejected.toArray(new Pose3d[0]));
  }

  @FunctionalInterface
  public static interface VisionConsumer {
    public void accept(
        Pose2d visionRobotPoseMeters,
        double timestampSeconds,
        Matrix<N3, N1> visionMeasurementStdDevs);
  }
}
