// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class FieldConstants {
  // all in meters
  public static final double FIELDLENGTH = 16.54099;
  public static final double FIELDWIDTH = 8.069326;

  public static final double ALLIANCEWALLTOHUB = 4.625594; // center of hub
  public static final Pose2d HUBPOSE =
      new Pose2d(ALLIANCEWALLTOHUB, FIELDWIDTH / 2.0, new Rotation2d()); // middle of hub

  public static final Pose2d DEPOTPOSE =
      new Pose2d(0.3429, 5.963158, new Rotation2d()); // middle of depot

  public static final double TRENCHWIDTH = 1.284986;

  public static final Pose2d TOPLEFTSIDETRENCH =
      new Pose2d(ALLIANCEWALLTOHUB, FIELDWIDTH, new Rotation2d());
  public static final Pose2d TOPRIGHTSIDETRENCH =
      new Pose2d(ALLIANCEWALLTOHUB, FIELDWIDTH - TRENCHWIDTH, new Rotation2d());

  public static final Pose2d BOTTOMLEFTSIDETRENCH =
      new Pose2d(ALLIANCEWALLTOHUB, TRENCHWIDTH, new Rotation2d());
  public static final Pose2d BOTTOMRIGHTSIDETRENCH =
      new Pose2d(ALLIANCEWALLTOHUB, 0.0, new Rotation2d());

  public static double BUMPWIDTH = 1.8542;
  public static double WALLWIDTH =
      0.3048; // width of the wall thing between the trench and the bump

  public static Pose2d TOPLEFTBUMP =
      new Pose2d(
          ALLIANCEWALLTOHUB,
          FIELDWIDTH - TRENCHWIDTH - WALLWIDTH,
          new Rotation2d()); // shoot at this to get fuel to our side
  public static Pose2d TOPRIGHTBUMP =
      new Pose2d(
          ALLIANCEWALLTOHUB, FIELDWIDTH - TRENCHWIDTH - WALLWIDTH - BUMPWIDTH, new Rotation2d());

  public static Pose2d BOTTOMLEFTBUMP =
      new Pose2d(
          ALLIANCEWALLTOHUB,
          TRENCHWIDTH + WALLWIDTH + BUMPWIDTH,
          new Rotation2d()); // or this one if we're closer
  public static Pose2d BOTTOMRIGHTBUMP =
      new Pose2d(ALLIANCEWALLTOHUB, TRENCHWIDTH + WALLWIDTH, new Rotation2d());
}
