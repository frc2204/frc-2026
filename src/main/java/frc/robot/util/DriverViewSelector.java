package frc.robot.util;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.cscore.HttpCamera.HttpCameraKind;
import edu.wpi.first.cscore.VideoSink;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.util.geometry.AllianceFlipUtil;
import java.util.function.Supplier;

public class DriverViewSelector {

  private static class CameraInfo {
    final String name;
    final HttpCamera httpCamera;
    final double mountYawDeg; // yaw relative to robot chassis (0 = forward, CCW positive)
    final double hFovDeg; // horizontal field of view
    final boolean isTurretMounted;

    CameraInfo(
        String name, String url, double mountYawDeg, double hFovDeg, boolean isTurretMounted) {
      this.name = name;
      this.httpCamera = new HttpCamera(name, url, HttpCameraKind.kMJPGStreamer);
      this.mountYawDeg = mountYawDeg;
      this.hFovDeg = hFovDeg;
      this.isTurretMounted = isTurretMounted;
    }
  }

  private static final double LL3_FOV_DEG = 62.5; // Limelight 3 horizontal FOV
  private static final double LL4_FOV_DEG = 82.0; // Limelight 4 horizontal FOV

  private final CameraInfo[] cameras;
  private final VideoSink switchedCamera;
  private final Supplier<Pose2d> poseSupplier;
  private final Supplier<Double> turretAngleDegSupplier;
  private int currentIndex = -1;

  public DriverViewSelector(
      Supplier<Pose2d> poseSupplier, Supplier<Double> turretAngleDegSupplier) {
    this.poseSupplier = poseSupplier;
    this.turretAngleDegSupplier = turretAngleDegSupplier;

    cameras =
        new CameraInfo[] {
          new CameraInfo(
              "limelight-left",
              "http://limelight-left.local:5800/stream.mjpg",
              45, // 11.75° left of forward
              LL3_FOV_DEG,
              false),
          new CameraInfo(
              "limelight-right",
              "http://limelight-right.local:5800/stream.mjpg",
              -45, // 11.75° right of forward
              LL3_FOV_DEG,
              false),
          new CameraInfo(
              "limelight-under",
              "http://limelight-under.local:5800/stream.mjpg",
              180.0, // faces backward
              LL4_FOV_DEG,
              false),
        };

    switchedCamera = CameraServer.addSwitchedCamera("Driver View");
    switchedCamera.setSource(cameras[0].httpCamera);
    currentIndex = 0;
  }

  /** Call once per cycle from updateDashboard(). */
  public void update() {
    Pose2d robotPose = poseSupplier.get();
    double turretAngleDeg = turretAngleDegSupplier.get();
    double robotHeadingDeg = robotPose.getRotation().getDegrees();

    // Alliance wall center: blue at x=0, red at x=FIELDLENGTH
    double wallX = AllianceFlipUtil.shouldFlip() ? FieldConstants.FIELDLENGTH : 0.0;
    double wallY = FieldConstants.FIELDWIDTH / 2.0;

    // Field-relative angle from robot to wall center
    double dx = wallX - robotPose.getX();
    double dy = wallY - robotPose.getY();
    double angleToWallDeg = Math.toDegrees(Math.atan2(dy, dx));

    // First pass: find best camera with wall in FOV
    int bestIndex = -1;
    double bestScore = Double.MAX_VALUE;

    for (int i = 0; i < cameras.length; i++) {
      double absError = getAngularError(i, robotHeadingDeg, turretAngleDeg, angleToWallDeg);

      if (absError <= cameras[i].hFovDeg / 2.0) {
        double score = absError - cameras[i].hFovDeg * 0.001; // tiebreak: wider FOV slightly better
        if (score < bestScore) {
          bestScore = score;
          bestIndex = i;
        }
      }
    }

    // Fallback: no camera has wall in FOV, pick smallest error
    if (bestIndex < 0) {
      for (int i = 0; i < cameras.length; i++) {
        double absError = getAngularError(i, robotHeadingDeg, turretAngleDeg, angleToWallDeg);
        if (absError < bestScore) {
          bestScore = absError;
          bestIndex = i;
        }
      }
    }

    // Switch only on change
    if (bestIndex != currentIndex) {
      switchedCamera.setSource(cameras[bestIndex].httpCamera);
      currentIndex = bestIndex;
    }

    SmartDashboard.putString("DriverView/ActiveCamera", cameras[currentIndex].name);
  }

  private double getAngularError(
      int cameraIndex, double robotHeadingDeg, double turretAngleDeg, double angleToWallDeg) {
    CameraInfo cam = cameras[cameraIndex];
    double camFieldYawDeg = robotHeadingDeg + cam.mountYawDeg;
    if (cam.isTurretMounted) {
      camFieldYawDeg += turretAngleDeg;
    }
    return Math.abs(MathUtil.inputModulus(angleToWallDeg - camFieldYawDeg, -180.0, 180.0));
  }
}
