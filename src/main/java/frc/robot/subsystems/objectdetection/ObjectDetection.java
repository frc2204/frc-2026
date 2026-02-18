package frc.robot.subsystems.objectdetection;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.RawDetection;
import frc.robot.util.FieldConstants;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class ObjectDetection extends SubsystemBase {

  private static ObjectDetection instance;

  public static ObjectDetection getInstance() {
    return instance;
  }

  // Camera config
  private static final String CAMERA_NAME = "limelight-detect";
  private static final int FUEL_CLASS_ID = 0;
  private static final double CAMERA_HEIGHT_METERS = 0.5461;
  private static final double TARGET_HEIGHT_METERS = 0.05715; // ball center on ground
  private static final double CAMERA_PITCH_DEGREES = -35.0;
  private static final Translation2d CAMERA_OFFSET_ROBOT = new Translation2d(0.0127, -0.1143);
  private static final double CAMERA_YAW_DEGREES = 0.0; // camera faces forward on robot

  // Detection filters
  private static final double MIN_TARGET_AREA = 0.1;
  private static final double MAX_DETECTION_DISTANCE = 6.0;
  private static final double MIN_DETECTION_DISTANCE = 0.15;
  private static final double FIELD_BOUNDARY_MARGIN = 0.3;

  // Tracking parameters
  private static final double DEDUP_DISTANCE_METERS = 0.3;
  private static final double EXPIRY_SECONDS = 2.0;
  private static final double PICKUP_RADIUS_METERS = 0.5;
  private static final double SMOOTHING_ALPHA = 0.7;

  private final Supplier<Pose2d> robotPoseSupplier;
  private final List<TrackedFuel> trackedFuels = new ArrayList<>();

  private static class TrackedFuel {
    Translation2d fieldPosition;
    double lastSeenTimestamp;

    TrackedFuel(Translation2d position, double timestamp) {
      this.fieldPosition = position;
      this.lastSeenTimestamp = timestamp;
    }

    void update(Translation2d newPosition, double timestamp) {
      this.fieldPosition =
          new Translation2d(
              fieldPosition.getX() * (1 - SMOOTHING_ALPHA) + newPosition.getX() * SMOOTHING_ALPHA,
              fieldPosition.getY() * (1 - SMOOTHING_ALPHA) + newPosition.getY() * SMOOTHING_ALPHA);
      this.lastSeenTimestamp = timestamp;
    }

    boolean isExpired(double currentTime) {
      return (currentTime - lastSeenTimestamp) > EXPIRY_SECONDS;
    }
  }

  public ObjectDetection(Supplier<Pose2d> robotPoseSupplier) {
    this.robotPoseSupplier = robotPoseSupplier;
    instance = this;
  }

  @Override
  public void periodic() {
    double now = Timer.getFPGATimestamp();
    Pose2d robotPose = robotPoseSupplier.get();

    // Read all neural network detections
    RawDetection[] rawDetections = LimelightHelpers.getRawDetections(CAMERA_NAME);

    // Convert valid detections to field-relative positions
    List<Translation2d> frameDetections = new ArrayList<>();
    for (RawDetection det : rawDetections) {
      if (det.classId != FUEL_CLASS_ID) continue;
      if (det.ta < MIN_TARGET_AREA) continue;

      Translation2d fieldPos = detectionToFieldPose(det.txnc, det.tync, robotPose);
      if (fieldPos != null) {
        frameDetections.add(fieldPos);
      }
    }

    // Merge new detections into tracked map
    mergeDetections(frameDetections, now);

    // Expire old detections
    trackedFuels.removeIf(tf -> tf.isExpired(now));

    // Remove balls near robot (already picked up)
    Translation2d robotTranslation = robotPose.getTranslation();
    trackedFuels.removeIf(
        tf -> tf.fieldPosition.getDistance(robotTranslation) < PICKUP_RADIUS_METERS);

    // Log for AdvantageScope
    logDetections(frameDetections, rawDetections.length);
  }

  private Translation2d detectionToFieldPose(double txncDeg, double tyncDeg, Pose2d robotPose) {
    // Distance from vertical angle
    double totalVerticalAngleRad = Math.toRadians(CAMERA_PITCH_DEGREES + tyncDeg);
    double heightDiff = CAMERA_HEIGHT_METERS - TARGET_HEIGHT_METERS;
    double distance = heightDiff / Math.tan(-totalVerticalAngleRad);

    if (distance < MIN_DETECTION_DISTANCE
        || distance > MAX_DETECTION_DISTANCE
        || Double.isNaN(distance)
        || Double.isInfinite(distance)) {
      return null;
    }

    // Camera-relative position
    double txRad = Math.toRadians(txncDeg);
    Translation2d cameraRelative =
        new Translation2d(distance * Math.cos(txRad), distance * Math.sin(txRad));

    // Rotate by fixed camera yaw, then add camera offset on robot
    Translation2d robotRelative =
        cameraRelative
            .rotateBy(Rotation2d.fromDegrees(CAMERA_YAW_DEGREES))
            .plus(CAMERA_OFFSET_ROBOT);

    // Robot-relative to field-relative
    Translation2d fieldRelative =
        robotRelative.rotateBy(robotPose.getRotation()).plus(robotPose.getTranslation());

    // Validate field boundaries
    if (fieldRelative.getX() < -FIELD_BOUNDARY_MARGIN
        || fieldRelative.getX() > FieldConstants.FIELDLENGTH + FIELD_BOUNDARY_MARGIN
        || fieldRelative.getY() < -FIELD_BOUNDARY_MARGIN
        || fieldRelative.getY() > FieldConstants.FIELDWIDTH + FIELD_BOUNDARY_MARGIN) {
      return null;
    }

    return fieldRelative;
  }

  private void mergeDetections(List<Translation2d> newDetections, double timestamp) {
    for (Translation2d newDet : newDetections) {
      TrackedFuel closest = null;
      double closestDist = Double.MAX_VALUE;

      for (TrackedFuel tracked : trackedFuels) {
        double dist = tracked.fieldPosition.getDistance(newDet);
        if (dist < closestDist) {
          closestDist = dist;
          closest = tracked;
        }
      }

      if (closest != null && closestDist < DEDUP_DISTANCE_METERS) {
        closest.update(newDet, timestamp);
      } else {
        trackedFuels.add(new TrackedFuel(newDet, timestamp));
      }
    }
  }

  private void logDetections(List<Translation2d> frameDetections, int rawCount) {
    Pose2d[] trackedPoses =
        trackedFuels.stream()
            .map(tf -> new Pose2d(tf.fieldPosition, new Rotation2d()))
            .toArray(Pose2d[]::new);
    Logger.recordOutput("ObjectDetection/TrackedFuel", trackedPoses);

    Pose2d[] framePoses =
        frameDetections.stream().map(t -> new Pose2d(t, new Rotation2d())).toArray(Pose2d[]::new);
    Logger.recordOutput("ObjectDetection/FrameDetections", framePoses);

    Logger.recordOutput("ObjectDetection/TrackedCount", trackedFuels.size());
    Logger.recordOutput("ObjectDetection/FrameRawCount", rawCount);

    Pose2d robotPose = robotPoseSupplier.get();
    Optional<Translation2d> nearest = getNearestFuel(robotPose.getTranslation());
    Logger.recordOutput(
        "ObjectDetection/NearestFuelDistance",
        nearest.map(n -> n.getDistance(robotPose.getTranslation())).orElse(-1.0));
  }

  /** Returns field-relative positions of all currently tracked FUEL balls. */
  public List<Translation2d> getFuelTranslations() {
    List<Translation2d> result = new ArrayList<>();
    for (TrackedFuel tf : trackedFuels) {
      result.add(tf.fieldPosition);
    }
    return result;
  }

  /** Returns the nearest tracked FUEL ball to the given position, or empty if none tracked. */
  public Optional<Translation2d> getNearestFuel(Translation2d position) {
    return trackedFuels.stream()
        .map(tf -> tf.fieldPosition)
        .min(Comparator.comparingDouble(pos -> pos.getDistance(position)));
  }

  /** Returns the number of currently tracked FUEL balls. */
  public int getTrackedFuelCount() {
    return trackedFuels.size();
  }

  /** Clears all tracked detections. */
  public void clearTracked() {
    trackedFuels.clear();
  }
}
