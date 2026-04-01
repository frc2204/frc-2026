// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

public class FieldConstants {
  // all in meters
  public static final double FIELDLENGTH = 16.54099;
  public static final double FIELDWIDTH = 8.069326;

  // Tower (blue side — use AllianceFlipUtil for red)
  public static final double TOWER_FRONT_X = Units.inchesToMeters(43.51); // front face from wall
  public static final double TOWER_BACK_X = 0.0; // against the alliance wall
  public static final double TOWER_WIDTH = Units.inchesToMeters(49.25);
  public static final double TOWER_CENTER_Y = 4.035; // centered on field
  public static final double TOWER_MIN_Y = TOWER_CENTER_Y - TOWER_WIDTH / 2.0;
  public static final double TOWER_MAX_Y = TOWER_CENTER_Y + TOWER_WIDTH / 2.0;

  /** Check if a robot pose is inside either alliance's tower footprint. */
  public static boolean isInsideTower(Pose2d pose) {
    double x = pose.getX();
    double y = pose.getY();
    // Blue tower
    if (x >= TOWER_BACK_X && x <= TOWER_FRONT_X && y >= TOWER_MIN_Y && y <= TOWER_MAX_Y) {
      return true;
    }
    // Red tower (mirrored)
    double redBackX = FIELDLENGTH - TOWER_FRONT_X;
    if (x >= redBackX && x <= FIELDLENGTH && y >= TOWER_MIN_Y && y <= TOWER_MAX_Y) {
      return true;
    }
    return false;
  }

  public static final Pose2d TOPTARGET =
      new Pose2d(0, FIELDWIDTH * 0.75, new Rotation2d()); // alliance wall at 3/4 field width
  public static final Pose2d BOTTOMTARGET =
      new Pose2d(0, FIELDWIDTH * 0.25, new Rotation2d()); // alliance wall at 1/4 field width

  public static final double ALLIANCEWALLTOHUB = 4.625594; // center of hub
  public static final Pose2d HUBPOSE =
      new Pose2d(ALLIANCEWALLTOHUB, FIELDWIDTH / 2.0, new Rotation2d()); // middle of hub

  public static final Pose2d DEPOTPOSE =
      new Pose2d(0.3429, 5.963158, new Rotation2d()); // middle of depot

  public static final Pose2d OUTPOSTPOSE =
      new Pose2d(0.0, 1.0, new Rotation2d()); // middle of outpost

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
      new Pose2d(ALLIANCEWALLTOHUB, FIELDWIDTH - TRENCHWIDTH - WALLWIDTH, new Rotation2d());
  public static Pose2d TOPRIGHTBUMP =
      new Pose2d(
          ALLIANCEWALLTOHUB, FIELDWIDTH - TRENCHWIDTH - WALLWIDTH - BUMPWIDTH, new Rotation2d());

  public static Pose2d BOTTOMLEFTBUMP =
      new Pose2d(ALLIANCEWALLTOHUB, TRENCHWIDTH + WALLWIDTH + BUMPWIDTH, new Rotation2d());
  public static Pose2d BOTTOMRIGHTBUMP =
      new Pose2d(ALLIANCEWALLTOHUB, TRENCHWIDTH + WALLWIDTH, new Rotation2d());

  // midpoint of bump zone for passing targets when on opposite alliance
  public static final Pose2d TOPBUMPMID =
      new Pose2d(
          ALLIANCEWALLTOHUB,
          FIELDWIDTH - TRENCHWIDTH - WALLWIDTH - BUMPWIDTH / 2.0,
          new Rotation2d());
  public static final Pose2d BOTTOMBUMPMID =
      new Pose2d(ALLIANCEWALLTOHUB, TRENCHWIDTH + WALLWIDTH + BUMPWIDTH / 2.0, new Rotation2d());
}
