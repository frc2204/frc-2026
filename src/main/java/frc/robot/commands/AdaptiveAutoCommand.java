package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
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
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;

public class AdaptiveAutoCommand extends Command {

  private enum State {
    FOLLOWING_PATH,
    CHASING_BALL,
    BLIND_DRIVE
  }

  // Tunable constants
  private static final double CHASE_SPEED = 2.0; // m/s
  private static final double BLIND_SPEED = 1.5; // m/s
  private static final double BLIND_DURATION = 1.0; // seconds
  private static final double CHASE_TIMEOUT = 3.0; // seconds
  private static final double WAYPOINT_TOLERANCE = 0.3; // meters
  private static final double HEADING_KP = 5.0;
  private static final double HEADING_KD = 0.4;
  private static final double HEADING_MAX_VEL = 8.0; // rad/s
  private static final double HEADING_MAX_ACCEL = 20.0; // rad/s^2
  private static final PathConstraints PATH_CONSTRAINTS =
      new PathConstraints(3.0, 3.0, 2 * Math.PI, 4 * Math.PI);

  private final Drive drive;
  private final ObjectDetection objectDetection;
  private final IntakeSubsystem intake;
  private final List<Pose2d> waypoints;

  private final ProfiledPIDController headingPID;
  private final Timer blindTimer = new Timer();
  private final Timer chaseTimer = new Timer();

  private State currentState;
  private int currentWaypointIndex;
  private Command activePathCommand;
  private Rotation2d lastChaseHeading;

  /** Create from a PathPlanner path file name. Samples the path into waypoints. */
  public AdaptiveAutoCommand(
      Drive drive, ObjectDetection objectDetection, IntakeSubsystem intake, String pathName) {
    this(drive, objectDetection, intake, loadPathPoses(pathName));
  }

  private static final double SAMPLE_SPACING_METERS = 0.5; // tune: distance between waypoints

  private static List<Pose2d> loadPathPoses(String pathName) {
    try {
      List<Pose2d> allPoses = PathPlannerPath.fromPathFile(pathName).getPathPoses();
      if (allPoses.isEmpty()) return List.of();

      // Sample at regular intervals so we don't get hundreds of tiny waypoints
      List<Pose2d> sampled = new ArrayList<>();
      sampled.add(allPoses.get(0));
      double accumulated = 0.0;
      for (int i = 1; i < allPoses.size(); i++) {
        accumulated +=
            allPoses.get(i).getTranslation().getDistance(allPoses.get(i - 1).getTranslation());
        if (accumulated >= SAMPLE_SPACING_METERS) {
          sampled.add(allPoses.get(i));
          accumulated = 0.0;
        }
      }
      // Always include the last pose
      Pose2d last = allPoses.get(allPoses.size() - 1);
      if (!sampled.get(sampled.size() - 1).equals(last)) {
        sampled.add(last);
      }
      return sampled;
    } catch (Exception e) {
      DriverStation.reportError("Failed to load path: " + pathName, e.getStackTrace());
      return List.of();
    }
  }

  /** Create from an explicit list of waypoints. */
  public AdaptiveAutoCommand(
      Drive drive,
      ObjectDetection objectDetection,
      IntakeSubsystem intake,
      List<Pose2d> waypoints) {
    this.drive = drive;
    this.objectDetection = objectDetection;
    this.intake = intake;
    this.waypoints = waypoints;

    headingPID =
        new ProfiledPIDController(
            HEADING_KP,
            0.0,
            HEADING_KD,
            new TrapezoidProfile.Constraints(HEADING_MAX_VEL, HEADING_MAX_ACCEL));
    headingPID.enableContinuousInput(-Math.PI, Math.PI);

    addRequirements(drive, intake);
  }

  @Override
  public void initialize() {
    currentState = State.FOLLOWING_PATH;
    currentWaypointIndex = 0;
    activePathCommand = null;
    lastChaseHeading = drive.getRotation();
    headingPID.reset(drive.getRotation().getRadians());
    blindTimer.stop();
    blindTimer.reset();
    chaseTimer.stop();
    chaseTimer.reset();

    startPathToWaypoint();
    intake.intake();
  }

  @Override
  public void execute() {
    Pose2d robotPose = drive.getPose();
    Optional<Translation2d> nearestBall =
        objectDetection.getNearestFuel(robotPose.getTranslation());

    Logger.recordOutput("AdaptiveAuto/State", currentState.name());
    Logger.recordOutput("AdaptiveAuto/WaypointIndex", currentWaypointIndex);
    Logger.recordOutput("AdaptiveAuto/WaypointCount", waypoints.size());
    Logger.recordOutput("AdaptiveAuto/BallDetected", nearestBall.isPresent());

    switch (currentState) {
      case FOLLOWING_PATH:
        executeFollowingPath(robotPose, nearestBall);
        break;
      case CHASING_BALL:
        executeChasingBall(robotPose, nearestBall);
        break;
      case BLIND_DRIVE:
        executeBlindDrive(robotPose, nearestBall);
        break;
    }
  }

  private void executeFollowingPath(Pose2d robotPose, Optional<Translation2d> nearestBall) {
    // Check if ball detected — switch to chase
    if (nearestBall.isPresent()) {
      cancelActivePath();
      switchToChasing();
      return;
    }

    // Check if we reached the current waypoint
    if (currentWaypointIndex < waypoints.size()) {
      double distToWaypoint =
          robotPose
              .getTranslation()
              .getDistance(waypoints.get(currentWaypointIndex).getTranslation());
      if (distToWaypoint < WAYPOINT_TOLERANCE) {
        currentWaypointIndex++;
        cancelActivePath();
        if (currentWaypointIndex < waypoints.size()) {
          startPathToWaypoint();
        }
      }
    }

    // Run active path command if it exists
    if (activePathCommand != null) {
      activePathCommand.execute();
      if (activePathCommand.isFinished()) {
        activePathCommand.end(false);
        currentWaypointIndex++;
        if (currentWaypointIndex < waypoints.size()) {
          startPathToWaypoint();
        }
      }
    }
  }

  private void executeChasingBall(Pose2d robotPose, Optional<Translation2d> nearestBall) {
    if (nearestBall.isEmpty()) {
      // Lost the ball — switch to blind drive
      switchToBlind();
      return;
    }

    // Safety timeout
    if (chaseTimer.hasElapsed(CHASE_TIMEOUT)) {
      switchToBlind();
      return;
    }

    // Drive toward ball
    Translation2d ballPos = nearestBall.get();
    Translation2d robotPos = robotPose.getTranslation();
    Translation2d toBall = ballPos.minus(robotPos);
    Rotation2d targetHeading = toBall.getAngle();
    lastChaseHeading = targetHeading;

    double omega =
        headingPID.calculate(drive.getRotation().getRadians(), targetHeading.getRadians());

    boolean isFlipped =
        DriverStation.getAlliance().isPresent()
            && DriverStation.getAlliance().get() == Alliance.Red;
    drive.runVelocity(
        ChassisSpeeds.fromFieldRelativeSpeeds(
            CHASE_SPEED * targetHeading.getCos(),
            CHASE_SPEED * targetHeading.getSin(),
            omega,
            isFlipped ? drive.getRotation().plus(new Rotation2d(Math.PI)) : drive.getRotation()));
  }

  private void executeBlindDrive(Pose2d robotPose, Optional<Translation2d> nearestBall) {
    // Ball reappeared — go back to chasing
    if (nearestBall.isPresent()) {
      switchToChasing();
      return;
    }

    // Blind timer expired — snap back to nearest waypoint on the path
    if (blindTimer.hasElapsed(BLIND_DURATION)) {
      drive.stop();
      snapBackToNearestWaypoint(robotPose);
      return;
    }

    // Keep driving forward at last heading, no rotation correction
    boolean isFlipped =
        DriverStation.getAlliance().isPresent()
            && DriverStation.getAlliance().get() == Alliance.Red;
    drive.runVelocity(
        ChassisSpeeds.fromFieldRelativeSpeeds(
            BLIND_SPEED * lastChaseHeading.getCos(),
            BLIND_SPEED * lastChaseHeading.getSin(),
            0.0,
            isFlipped ? drive.getRotation().plus(new Rotation2d(Math.PI)) : drive.getRotation()));
  }

  private void switchToChasing() {
    currentState = State.CHASING_BALL;
    chaseTimer.restart();
    headingPID.reset(drive.getRotation().getRadians());
  }

  private void switchToBlind() {
    currentState = State.BLIND_DRIVE;
    blindTimer.restart();
  }

  private void snapBackToNearestWaypoint(Pose2d robotPose) {
    // Find closest waypoint at or ahead of current index
    int bestIndex = currentWaypointIndex;
    double bestDist = Double.MAX_VALUE;
    for (int i = currentWaypointIndex; i < waypoints.size(); i++) {
      double d = robotPose.getTranslation().getDistance(waypoints.get(i).getTranslation());
      if (d < bestDist) {
        bestDist = d;
        bestIndex = i;
      }
    }
    currentWaypointIndex = bestIndex;

    if (currentWaypointIndex < waypoints.size()) {
      startPathToWaypoint();
    }
    currentState = State.FOLLOWING_PATH;
  }

  private void startPathToWaypoint() {
    cancelActivePath();
    if (currentWaypointIndex >= waypoints.size()) return;

    Pose2d target = waypoints.get(currentWaypointIndex);
    activePathCommand = AutoBuilder.pathfindToPose(target, PATH_CONSTRAINTS, 0.0);
    activePathCommand.initialize();
  }

  private void cancelActivePath() {
    if (activePathCommand != null) {
      activePathCommand.end(true);
      activePathCommand = null;
    }
  }

  @Override
  public boolean isFinished() {
    return currentState == State.FOLLOWING_PATH
        && currentWaypointIndex >= waypoints.size()
        && activePathCommand == null;
  }

  @Override
  public void end(boolean interrupted) {
    cancelActivePath();
    drive.stop();
    intake.stow();
  }
}
