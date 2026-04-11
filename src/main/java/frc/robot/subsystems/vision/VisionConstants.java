// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;

/*
 * Vision tuning constants for multi-camera AprilTag pose estimation.
 *
 * ARCHITECTURE OVERVIEW:
 * - 3 Limelight cameras feed pose observations into WPILib's SwerveDrivePoseEstimator
 * - Each observation gets a standard deviation (trust level) that scales with distance, tag count,
 *   camera quality, and robot rotation rate
 * - Observations are rejected outright if they fail sanity checks (ambiguity, field bounds, etc.)
 * - Only MegaTag 2 (MT2) is consumed; MT1 is skipped (see VisionIOLimelight.java for why)
 *
 * ANTI-OSCILLATION STRATEGY (researched from 254, 6328, 1678, 3847 public code):
 * The main cause of oscillation is multiple cameras disagreeing on the robot's pose and the
 * estimator ping-ponging between them. Three defenses:
 *   1. Pose sanity check (maxPoseDisagreement) — reject observations far from current estimate
 *   2. Hard gyro rate rejection (maxGyroRateForVision) — rolling shutter frames are useless when
 *      spinning
 *   3. Camera trust differentiation (cameraStdDevFactors) — LL4 global shutter gets much more
 *      weight than LL3 rolling shutter
 *
 * STD DEV FORMULA (computed in Vision.java):
 *   linearStdDev = linearStdDevBaseline
 *                  * (averageTagDistance ^ distanceExponent)
 *                  / (tagCount ^ 2)
 *                  * megatag2Factor
 *                  * cameraStdDevFactor
 *                  * omegaFactor
 *   angularStdDev = same formula, but forced to INFINITY for single-tag observations
 *
 * TUNING GUIDE:
 * - If pose oscillates when standing still with 2+ cameras → lower maxPoseDisagreement or raise
 *   linearStdDevBaseline
 * - If pose feels laggy / doesn't correct fast enough → lower linearStdDevBaseline or lower
 *   camera stddev factors
 * - If pose jumps wildly during fast spins → lower maxGyroRateForVision
 * - If single-tag observations at distance cause drift → lower maxSingleTagDistance
 * - To debug: check AdvantageScope Vision/CameraX/RobotPosesAccepted vs RobotPosesRejected
 *
 * REFERENCE VALUES FROM TOP TEAMS (2024-2025 season):
 *   6328: linearBaseline=0.01, distExp=1.2, tagCount^2 divisor, camera factors [1.0, 0.6, 1.0, 1.2]
 *   254:  reads stddevs from Limelight firmware, ambiguity threshold 0.19
 *   1678: linearBaseline~0.3*dist (2025), never trusts rotation, velocity>4m/s rejects
 *   3847: tier-based stddevs, omega>1.6 rejects all, poseDiff<0.5m required
 */
public class VisionConstants {
  // AprilTag layout
  public static AprilTagFieldLayout aprilTagLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

  // Camera names, must match names configured on coprocessor.
  // Camera indices in Vision.java correspond to the order passed to the Vision constructor
  // in RobotContainer.java — currently: [0]=limelight-left, [1]=limelight-right,
  // [2]=limelight-under.
  // NOTE: limelight-turret (camera0Name) is commented out in RobotContainer and not used for pose
  // estimation — it's used by TurretSubsystem for direct target tracking (tx/ty).
  public static String camera0Name = "limelight-turret";
  public static String camera1Name = "limelight-left";
  public static String camera2Name = "limelight-right";
  public static String camera3Name = "limelight-under";

  // Robot-to-camera transforms (NOT used by Limelight — configure in the Limelight web UI instead).
  // These are only used by PhotonVision in simulation.
  // If cameras disagree on pose by a consistent offset, the transforms in the Limelight web UI
  // are miscalibrated — re-measure the physical camera positions on the robot.
  public static Transform3d robotToCamera0 =
      new Transform3d(0.2, 0.0, 0.2, new Rotation3d(0.0, -0.4, 0.0));
  public static Transform3d robotToCamera1 =
      new Transform3d(-0.2, 0.0, 0.2, new Rotation3d(0.0, -0.4, Math.PI));

  // --- Rejection thresholds (observations failing these are discarded entirely) ---

  // Maximum PnP ambiguity for single-tag observations. Lower = stricter.
  // 254 uses 0.19, 6328 uses 0.4, 3847 uses 0.9. We use 0.2 (moderately strict).
  public static double maxAmbiguity = 0.2;

  // Maximum Z-coordinate error (meters). A correct observation should have Z near 0.
  public static double maxZError = 0.75;

  // Maximum distance (meters) to accept a single-tag observation from LL3 cameras.
  // Single-tag PnP degrades rapidly with distance; multi-tag is fine at longer range.
  public static double maxSingleTagDistance = 3.5;

  // Same threshold for LL4 (global shutter = less rolling-shutter distortion = reliable farther).
  // Cameras with cameraStdDevFactors < 1.0 are treated as LL4-class.
  public static double maxSingleTagDistanceLL4 = 5.0;

  // Reject any observation whose 2D position is more than this many meters from the current
  // estimated pose. Prevents a single bad camera frame from yanking the estimator.
  // 254 uses yaw-diff + tag-area checks; 3847 uses 0.25-0.5m; 1678 uses 2.0m in auto.
  // 1.0m is a safe default — loose enough for legitimate corrections, tight enough to prevent
  // ping-ponging. Lower to 0.5 if oscillation persists; raise to 2.0 if match-start localization
  // is getting rejected.
  public static double maxPoseDisagreement = 1.0;

  // Reject ALL vision observations when the robot is spinning faster than this (rad/s).
  // Rolling shutter cameras (LL3) produce extremely blurry frames above ~1 rad/s.
  // 3847 uses 1.6 rad/s. Below this threshold, stddevs are still scaled up by omega (see
  // Vision.java) — this is the hard cutoff where vision data is completely unreliable.
  public static double maxGyroRateForVision = 1.5;

  // --- Standard deviation baselines ---
  // These are the base trust values at 1 meter distance with 1 tag visible.
  // They get scaled up by distance, scaled down by tag count, and modified by camera/MT2 factors.
  // Higher = less trust = more estimator smoothing. Lower = more trust = faster corrections.

  // Linear (XY translation) std dev baseline in meters.
  // 6328 uses 0.01 (very aggressive); we use 0.04 (moderate, reduces multi-camera oscillation).
  public static double linearStdDevBaseline = 0.04;

  // Angular (rotation) std dev baseline in radians.
  // Note: for single-tag observations, angular stddev is forced to INFINITY in Vision.java
  // regardless of this value — single-tag rotation estimates are unreliable.
  public static double angularStdDevBaseline = 0.1;

  // Exponent for distance scaling: stddev *= distance^exponent.
  // Higher = trusts distant tags less. 6328 uses 1.2, 1678 (2024) uses 2.0.
  // 1.5 is a moderate choice — 2.0 is too aggressive (kills useful distant multi-tag).
  public static double distanceExponent = 1.5;

  // --- Per-camera trust multipliers ---
  // Applied after baseline + distance + tag count scaling. Lower = more trusted.
  // Array indices correspond to the order cameras are passed to Vision() in RobotContainer:
  //   [0] = limelight-left (LL3, rolling shutter)
  //   [1] = limelight-right (LL3, rolling shutter)
  //   [2] = limelight-under (LL4, global shutter)
  // LL4 global shutter produces sharper frames and more accurate poses than LL3 rolling shutter,
  // so it gets a much lower factor. The 3:1 ratio (1.5 vs 0.5) means the estimator weights LL4
  // observations ~3x more heavily than LL3 when both see tags.
  public static double[] cameraStdDevFactors =
      new double[] {
        1.5, // limelight-left (LL3, rolling shutter)
        1.5, // limelight-right (LL3, rolling shutter)
        0.5 // limelight-under (LL4, global shutter — most trustworthy)
      };

  // --- MegaTag 2 multipliers ---
  // MT2 constrains rotation from the gyro and only solves for translation, making it more stable
  // than MT1's full 3D PnP solve. These factors adjust stddevs for MT2 observations.

  // Linear stddev multiplier for MT2. 0.5 = MT2 translation is ~2x more trusted than MT1.
  public static double linearStdDevMegatag2Factor = 0.5;

  // Angular stddev multiplier for MT2. INFINITY = never trust MT2 rotation (it uses the gyro's
  // rotation as input, so feeding it back would be circular). This is standard — 1678, 3847,
  // and the AdvantageKit template all set this to infinity.
  public static double angularStdDevMegatag2Factor = Double.POSITIVE_INFINITY;
}
