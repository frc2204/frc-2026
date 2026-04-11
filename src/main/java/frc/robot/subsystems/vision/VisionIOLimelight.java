// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotController;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Set;
import java.util.function.Supplier;

/**
 * IO implementation for real Limelight hardware. Reads AprilTag pose observations via
 * NetworkTables.
 *
 * <p>MEGATAG 1 vs MEGATAG 2:
 *
 * <ul>
 *   <li>MT1 ("botpose_wpiblue"): Full 3D PnP solve. Computes both translation AND rotation from tag
 *       geometry alone. More noisy, especially with rolling shutter (LL3). Can have high ambiguity
 *       with single tags.
 *   <li>MT2 ("botpose_orb_wpiblue"): Gyro-constrained solve. We send the gyro rotation to the
 *       Limelight via "robot_orientation_set", and MT2 uses it as a constraint — it only solves for
 *       translation. This produces more stable poses because the gyro rotation is much more
 *       accurate than PnP rotation.
 * </ul>
 *
 * <p>WHY WE ONLY USE MT2: Using both MT1 and MT2 from the same camera causes the pose estimator to
 * oscillate — they compute slightly different poses (different solve methods), and feeding both
 * gives the Kalman filter conflicting data every cycle. MT2 is strictly more stable for
 * translation, and we don't need MT1's rotation (the gyro is better). This matches 1678's 2025
 * approach (MT2 only).
 *
 * <p>The MT1 subscriber is still read (readQueue) to drain the buffer and extract tag IDs for
 * logging, but its pose observations are not consumed.
 */
public class VisionIOLimelight implements VisionIO {
  private final Supplier<Rotation2d> rotationSupplier;
  private final DoubleArrayPublisher orientationPublisher;

  private final DoubleSubscriber latencySubscriber;
  private final DoubleSubscriber txSubscriber;
  private final DoubleSubscriber tySubscriber;
  private final DoubleArraySubscriber megatag1Subscriber; // Drained but not consumed (see above)
  private final DoubleArraySubscriber megatag2Subscriber; // Primary pose source

  /**
   * Creates a new VisionIOLimelight.
   *
   * @param name The configured name of the Limelight.
   * @param rotationSupplier Supplier for the current estimated rotation, used for MegaTag 2.
   */
  public VisionIOLimelight(String name, Supplier<Rotation2d> rotationSupplier) {
    var table = NetworkTableInstance.getDefault().getTable(name);
    this.rotationSupplier = rotationSupplier;
    orientationPublisher = table.getDoubleArrayTopic("robot_orientation_set").publish();
    latencySubscriber = table.getDoubleTopic("tl").subscribe(0.0);
    txSubscriber = table.getDoubleTopic("tx").subscribe(0.0);
    tySubscriber = table.getDoubleTopic("ty").subscribe(0.0);
    megatag1Subscriber = table.getDoubleArrayTopic("botpose_wpiblue").subscribe(new double[] {});
    megatag2Subscriber =
        table.getDoubleArrayTopic("botpose_orb_wpiblue").subscribe(new double[] {});
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    // Update connection status based on whether an update has been seen in the last
    // 250ms
    inputs.connected =
        ((RobotController.getFPGATime() - latencySubscriber.getLastChange()) / 1000) < 250;

    // Update target observation
    inputs.latestTargetObservation =
        new TargetObservation(
            Rotation2d.fromDegrees(txSubscriber.get()), Rotation2d.fromDegrees(tySubscriber.get()));

    // Send the current gyro rotation to the Limelight for MegaTag 2.
    // MT2 uses this as a rotation constraint — it fixes the rotation and only solves for
    // translation, which is why MT2 poses are more stable than MT1. The array format is
    // [yaw, yawRate, pitch, pitchRate, roll, rollRate] — we only send yaw (degrees).
    orientationPublisher.accept(
        new double[] {rotationSupplier.get().getDegrees(), 0.0, 0.0, 0.0, 0.0, 0.0});
    NetworkTableInstance.getDefault()
        .flush(); // Increases network traffic but recommended by Limelight

    // Read new pose observations from NetworkTables
    Set<Integer> tagIds = new HashSet<>();
    List<PoseObservation> poseObservations = new LinkedList<>();
    // MT1 (full 3D PnP) is skipped — MT2 is more stable for pose estimation because it
    // constrains rotation from the gyro. Using both MT1+MT2 causes the estimator to oscillate
    // when they disagree. Only drain the queue so it doesn't build up stale data.
    for (var rawSample : megatag1Subscriber.readQueue()) {
      if (rawSample.value.length < 11) continue;
      for (int i = 11; i < rawSample.value.length; i += 7) {
        tagIds.add((int) rawSample.value[i]);
      }
      // Tag IDs still tracked for logging, but pose is not consumed.
    }
    for (var rawSample : megatag2Subscriber.readQueue()) {
      if (rawSample.value.length < 11) continue;
      for (int i = 11; i < rawSample.value.length; i += 7) {
        tagIds.add((int) rawSample.value[i]);
      }
      poseObservations.add(
          new PoseObservation(
              // Timestamp, based on server timestamp of publish and latency
              rawSample.timestamp * 1.0e-6 - rawSample.value[6] * 1.0e-3,

              // 3D pose estimate
              parsePose(rawSample.value),

              // Ambiguity, zeroed because the pose is already disambiguated
              0.0,

              // Tag count
              (int) rawSample.value[7],

              // Average tag distance
              rawSample.value[9],

              // Observation type
              PoseObservationType.MEGATAG_2));
    }

    // Save pose observations to inputs object
    inputs.poseObservations = new PoseObservation[poseObservations.size()];
    for (int i = 0; i < poseObservations.size(); i++) {
      inputs.poseObservations[i] = poseObservations.get(i);
    }

    // Save tag IDs to inputs objects
    inputs.tagIds = new int[tagIds.size()];
    int i = 0;
    for (int id : tagIds) {
      inputs.tagIds[i++] = id;
    }
  }

  /** Parses the 3D pose from a Limelight botpose array. */
  private static Pose3d parsePose(double[] rawLLArray) {
    return new Pose3d(
        rawLLArray[0],
        rawLLArray[1],
        rawLLArray[2],
        new Rotation3d(
            Units.degreesToRadians(rawLLArray[3]),
            Units.degreesToRadians(rawLLArray[4]),
            Units.degreesToRadians(rawLLArray[5])));
  }
}
