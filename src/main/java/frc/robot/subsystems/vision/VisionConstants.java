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

public class VisionConstants {
  // AprilTag layout
  public static AprilTagFieldLayout aprilTagLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

  // Camera names, must match names configured on coprocessor
  public static String camera0Name = "limelight-turret";
  public static String camera1Name = "limelight-left";
  public static String camera2Name = "limelight-right";
  public static String camera3Name = "limelight-under";

  // Robot to camera transforms
  // (Not used by Limelight, configure in web UI instead)
  public static Transform3d robotToCamera0 =
      new Transform3d(0.2, 0.0, 0.2, new Rotation3d(0.0, -0.4, 0.0));
  public static Transform3d robotToCamera1 =
      new Transform3d(-0.2, 0.0, 0.2, new Rotation3d(0.0, -0.4, Math.PI));

  // Basic filtering thresholds
  public static double maxAmbiguity = 0.3;
  public static double maxZError = 0.75;
  public static double maxSingleTagDistance = 3.5; // LL3 reliable range
  public static double maxSingleTagDistanceLL4 = 5.0; // LL4 global shutter = better at distance

  // Standard deviation baselines, for 1 meter distance and 1 tag
  // (Adjusted automatically based on distance and # of tags)
  public static double linearStdDevBaseline = 0.02; // Meters
  public static double angularStdDevBaseline = 0.06; // Radians

  // Distance exponent for std dev scaling (6328 uses 1.2, 2.0 is too aggressive)
  public static double distanceExponent = 1.5;

  // Standard deviation multipliers for each camera
  // Camera order: limelight-left (LL3), limelight-right (LL3), limelight-under (LL4)
  public static double[] cameraStdDevFactors =
      new double[] {
        1.0, // limelight-left (LL3, baseline)
        1.0, // limelight-right (LL3, baseline)
        0.8 // limelight-under (LL4, global shutter = more trustworthy)
      };

  // Multipliers to apply for MegaTag 2 observations
  public static double linearStdDevMegatag2Factor = 0.5; // More stable than full 3D solve
  public static double angularStdDevMegatag2Factor =
      Double.POSITIVE_INFINITY; // No rotation data available
}
