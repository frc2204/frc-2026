package frc.robot.util;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ObjectToPose extends SubsystemBase {

  private final NetworkTable limelight;

  private static final double CAMERA_HEIGHT = 0.5461; // meters
  private static final double TARGET_HEIGHT = 0.05715; // meters (ball center)
  private static final double CAMERA_PITCH = -35.0; // degrees

  public ObjectToPose() {
    limelight = NetworkTableInstance.getDefault().getTable("limelight");
  }

  //    public Optional<Pose2d> getBallFieldPose() {
  public void getBallFieldPose() {
    double tv = limelight.getEntry("tv").getDouble(0);
    if (tv == 0) {
      //            return Optional.empty(); // no target
    }

    double tx = limelight.getEntry("tx").getDouble(0.0);
    double ty = limelight.getEntry("ty").getDouble(0.0);

    // ===== Distance Calculation =====
    double angleDegrees = CAMERA_PITCH + ty;
    double angleRadians = Math.toRadians(angleDegrees);

    double distance = (TARGET_HEIGHT - CAMERA_HEIGHT) / Math.tan(angleRadians);
    distance = Math.abs(distance);

    // ===== Robot-relative translation =====
    double horizontalRadians = Math.toRadians(tx);

    double x = distance * Math.cos(horizontalRadians);
    double y = distance * Math.sin(horizontalRadians);

    Translation2d ballRelative = new Translation2d(x, y);
    System.out.println(Units.metersToInches(x) + " " + Units.metersToInches(y));
    //        double[] botpose = limelight
    //                .getEntry("botpose_wpiblue")
    //                .getDoubleArray(new double[7]);
    //
    //        if (botpose.length < 6) {
    //            return Optional.empty();
    //        }
    //
    //        Pose2d robotPose = new Pose2d(
    //                botpose[0],
    //                botpose[1],
    //                Rotation2d.fromDegrees(botpose[5])
    //        );
    //
    //        // ===== Transform to field =====
    //        Pose2d ballFieldPose = robotPose.transformBy(
    //                new Transform2d(ballRelative, new Rotation2d())
    //        );
    //
    //        return Optional.of(ballFieldPose);
  }

  @Override
  public void periodic() {
    getBallFieldPose();
  }
}
