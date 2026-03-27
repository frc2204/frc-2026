package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.objectdetection.ObjectDetection;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;

public class ChaseBallCommand extends Command {

  private static final double CHASE_SPEED = 2.5; // m/s
  private static final double BLIND_SPEED = 1.5; // m/s
  private static final double BLIND_DURATION = 1.0; // seconds
  private static final double HEADING_KP = 5.0;
  private static final double HEADING_KD = 0.4;

  private final Drive drive;
  private final ObjectDetection objectDetection;
  private final IntakeSubsystem intake;

  private final ProfiledPIDController headingPID;
  private final Timer blindTimer = new Timer();

  private boolean isBlind;
  private Rotation2d lastChaseHeading;

  public ChaseBallCommand(Drive drive, ObjectDetection objectDetection, IntakeSubsystem intake) {
    this.drive = drive;
    this.objectDetection = objectDetection;
    this.intake = intake;

    headingPID =
        new ProfiledPIDController(
            HEADING_KP, 0.0, HEADING_KD, new TrapezoidProfile.Constraints(8.0, 20.0));
    headingPID.enableContinuousInput(-Math.PI, Math.PI);

    addRequirements(drive, intake);
  }

  @Override
  public void initialize() {
    isBlind = false;
    lastChaseHeading = drive.getRotation();
    headingPID.reset(drive.getRotation().getRadians());
    blindTimer.stop();
    blindTimer.reset();
    intake.intake();
  }

  @Override
  public void execute() {
    Pose2d robotPose = drive.getPose();
    Optional<Translation2d> nearestBall =
        objectDetection.getNearestFuel(robotPose.getTranslation());

    Logger.recordOutput("ChaseBall/BallDetected", nearestBall.isPresent());
    Logger.recordOutput("ChaseBall/Blind", isBlind);

    boolean isFlipped =
        DriverStation.getAlliance().isPresent()
            && DriverStation.getAlliance().get() == Alliance.Red;

    if (nearestBall.isPresent()) {
      // See a ball — chase it
      isBlind = false;
      blindTimer.reset();

      Translation2d toBall = nearestBall.get().minus(robotPose.getTranslation());
      Rotation2d targetHeading = toBall.getAngle();
      lastChaseHeading = targetHeading;

      double omega =
          headingPID.calculate(drive.getRotation().getRadians(), targetHeading.getRadians());

      drive.runVelocity(
          ChassisSpeeds.fromFieldRelativeSpeeds(
              CHASE_SPEED * targetHeading.getCos(),
              CHASE_SPEED * targetHeading.getSin(),
              omega,
              isFlipped ? drive.getRotation().plus(new Rotation2d(Math.PI)) : drive.getRotation()));
    } else if (!isBlind) {
      // Just lost the ball — start blind drive
      isBlind = true;
      blindTimer.restart();
      driveBlind(isFlipped);
    } else if (!blindTimer.hasElapsed(BLIND_DURATION)) {
      // Still in blind drive window
      driveBlind(isFlipped);
    } else {
      // Blind timer expired, no ball — stop
      drive.stop();
    }
  }

  private void driveBlind(boolean isFlipped) {
    drive.runVelocity(
        ChassisSpeeds.fromFieldRelativeSpeeds(
            BLIND_SPEED * lastChaseHeading.getCos(),
            BLIND_SPEED * lastChaseHeading.getSin(),
            0.0,
            isFlipped ? drive.getRotation().plus(new Rotation2d(Math.PI)) : drive.getRotation()));
  }

  @Override
  public void end(boolean interrupted) {
    drive.stop();
    intake.stow();
  }

  @Override
  public boolean isFinished() {
    return false; // runs until cancelled
  }
}
