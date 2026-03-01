// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import static frc.robot.subsystems.vision.VisionConstants.*;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.AdaptiveAutoCommand;
import frc.robot.commands.ChaseBallCommand;
import frc.robot.commands.DriveCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.*;
import frc.robot.subsystems.handoff.HandoffSubsystem;
import frc.robot.subsystems.intake.IndexerSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.objectdetection.ObjectDetection;
import frc.robot.subsystems.shooting.HoodSubsystem;
import frc.robot.subsystems.shooting.ShooterSubsystem;
import frc.robot.subsystems.shooting.TurretSubsystem;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOLimelight;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
import frc.robot.util.FieldConstants;
import frc.robot.util.HubShiftUtil;
import frc.robot.util.geometry.AllianceFlipUtil;
import java.util.List;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive drive;
  private final Vision vision;
  private final ShooterSubsystem shooter = ShooterSubsystem.getInstance();
  private final IntakeSubsystem intake = IntakeSubsystem.getInstance();
  private final IndexerSubsystem indexer = IndexerSubsystem.getInstance();
  private final HandoffSubsystem handoff = HandoffSubsystem.getInstance();
  private final HoodSubsystem hood = HoodSubsystem.getInstance();
  private final TurretSubsystem turret;
  private final ObjectDetection objectDetection;
  private final frc.robot.subsystems.leds.CANdleSubsystem leds;
  private final frc.robot.util.DriverViewSelector driverView;
  private final PowerDistribution pdh = new PowerDistribution(45, ModuleType.kRev);

  // Controller
  private final CommandPS5Controller ps5Controller = new CommandPS5Controller(0);
  //  private final CommandPS5Controller ps5Controller = new CommandPS5Controller(1);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;
  private final Field2d field = new Field2d();
  private int pdhLogCounter = 0;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                new ModuleIOTalonFX(TunerConstants.FrontRight),
                new ModuleIOTalonFX(TunerConstants.BackLeft),
                new ModuleIOTalonFX(TunerConstants.BackRight));

        vision =
            new Vision(
                drive::addVisionMeasurement,
                //                new VisionIOLimelight(camera0Name, drive::getRotation),
                new VisionIOLimelight(camera1Name, drive::getRotation),
                new VisionIOLimelight(camera2Name, drive::getRotation),
                new VisionIOLimelight(camera3Name, drive::getRotation));
        turret = new TurretSubsystem(drive::getPose, drive::getChassisSpeeds);
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(TunerConstants.FrontLeft),
                new ModuleIOSim(TunerConstants.FrontRight),
                new ModuleIOSim(TunerConstants.BackLeft),
                new ModuleIOSim(TunerConstants.BackRight));

        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOPhotonVisionSim(camera0Name, robotToCamera0, drive::getPose));
        turret = new TurretSubsystem(drive::getPose, drive::getChassisSpeeds);
        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});

        vision = new Vision(drive::addVisionMeasurement, new VisionIO() {}, new VisionIO() {});
        turret = new TurretSubsystem(drive::getPose, drive::getChassisSpeeds);
        break;
    }

    // Set up object detection
    objectDetection = new ObjectDetection(drive::getPose);

    leds = new frc.robot.subsystems.leds.CANdleSubsystem(turret);
    driverView =
        new frc.robot.util.DriverViewSelector(drive::getPose, turret::getAbsolutePositionDeg);

    // Register named commands for PathPlanner
    com.pathplanner.lib.auto.NamedCommands.registerCommand(
        "deployIntake", Commands.runOnce(() -> intake.intake(), intake));
    com.pathplanner.lib.auto.NamedCommands.registerCommand(
        "stowIntake", Commands.runOnce(() -> intake.stow(), intake));
    com.pathplanner.lib.auto.NamedCommands.registerCommand(
        "spinUpShooter",
        Commands.runOnce(() -> shooter.setState(ShooterSubsystem.ShooterState.SPIN_UP), shooter));
    com.pathplanner.lib.auto.NamedCommands.registerCommand(
        "idleShooter",
        Commands.runOnce(() -> shooter.setState(ShooterSubsystem.ShooterState.IDLE), shooter));
    com.pathplanner.lib.auto.NamedCommands.registerCommand(
        "overrideShooter",
        Commands.runOnce(() -> shooter.setState(ShooterSubsystem.ShooterState.OVERRIDE), shooter));
    com.pathplanner.lib.auto.NamedCommands.registerCommand(
        "rapidFireShooter",
        Commands.runOnce(
            () -> shooter.setState(ShooterSubsystem.ShooterState.RAPID_FIRE), shooter));

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up SysId routines
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    // Adaptive ball-chasing auto — draw "Ball Sweep" path in PathPlanner
    autoChooser.addOption(
        "Adaptive Ball Auto",
        new AdaptiveAutoCommand(drive, objectDetection, intake, "Ball Sweep"));

    // Simple ball chase — just drives at any detected ball
    autoChooser.addOption("Chase Ball", new ChaseBallCommand(drive, objectDetection, intake));

    // Elastic dashboard setup
    SmartDashboard.putData("Field", field);
    SmartDashboard.putData(
        "Swerve Drive",
        new Sendable() {
          @Override
          public void initSendable(SendableBuilder builder) {
            builder.setSmartDashboardType("SwerveDrive");
            builder.addDoubleProperty(
                "Front Left Angle", () -> drive.getModuleStates()[0].angle.getRadians(), null);
            builder.addDoubleProperty(
                "Front Left Velocity", () -> drive.getModuleStates()[0].speedMetersPerSecond, null);
            builder.addDoubleProperty(
                "Front Right Angle", () -> drive.getModuleStates()[1].angle.getRadians(), null);
            builder.addDoubleProperty(
                "Front Right Velocity",
                () -> drive.getModuleStates()[1].speedMetersPerSecond,
                null);
            builder.addDoubleProperty(
                "Back Left Angle", () -> drive.getModuleStates()[2].angle.getRadians(), null);
            builder.addDoubleProperty(
                "Back Left Velocity", () -> drive.getModuleStates()[2].speedMetersPerSecond, null);
            builder.addDoubleProperty(
                "Back Right Angle", () -> drive.getModuleStates()[3].angle.getRadians(), null);
            builder.addDoubleProperty(
                "Back Right Velocity", () -> drive.getModuleStates()[3].speedMetersPerSecond, null);
            builder.addDoubleProperty("Robot Angle", () -> drive.getRotation().getRadians(), null);
          }
        });

    // Turret angle as radial gauge
    SmartDashboard.putData(
        "Turret Angle",
        new Sendable() {
          @Override
          public void initSendable(SendableBuilder builder) {
            builder.setSmartDashboardType("Gyro");
            builder.addDoubleProperty("Value", turret::getAbsolutePositionDeg, null);
          }
        });

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Default command, field-relative drive with shooting speed limit
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -ps5Controller.getLeftY() * getShootingSpeedFactor(),
            () -> -ps5Controller.getLeftX() * getShootingSpeedFactor(),
            () -> -ps5Controller.getRightX() * getShootingSpeedFactor()));

    // Square: override speed limit (full speed while held)
    ps5Controller
        .square()
        .whileTrue(
            DriveCommands.joystickDrive(
                drive,
                () -> -ps5Controller.getLeftY(),
                () -> -ps5Controller.getLeftX(),
                () -> -ps5Controller.getRightX()));

    double slowFactor = 0.65; // tune
    ps5Controller
        .cross()
        .whileTrue(
            DriveCommands.joystickDrive(
                drive,
                () -> -ps5Controller.getLeftY() * slowFactor,
                () -> -ps5Controller.getLeftX() * slowFactor,
                () -> -ps5Controller.getRightX() * slowFactor));

    // Options: reset gyro to 0°
    ps5Controller
        .options()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), Rotation2d.kZero)),
                    drive)
                .ignoringDisable(true));

    // Triangle: spin up shooter
    ps5Controller
        .triangle()
        .onTrue(
            Commands.runOnce(
                () -> shooter.setState(ShooterSubsystem.ShooterState.SPIN_UP), shooter));

    // Circle: idle shooter
    ps5Controller
        .circle()
        .onTrue(
            Commands.runOnce(() -> shooter.setState(ShooterSubsystem.ShooterState.IDLE), shooter));

    // R2 or L2: either trigger activates shooting system
    // Both together: force fire override
    ps5Controller
        .R2()
        .or(ps5Controller.L2())
        .whileTrue(
            Commands.run(
                () -> {
                  boolean r2Held = ps5Controller.R2().getAsBoolean();
                  boolean l2Held = ps5Controller.L2().getAsBoolean();
                  boolean override = r2Held && l2Held;
                  boolean shiftActive =
                      !edu.wpi.first.wpilibj.DriverStation.isFMSAttached()
                          || HubShiftUtil.getShiftedShiftInfo().active();
                  boolean onTarget = turret.isOnTarget();
                  boolean passing = turret.isPassingMode();
                  boolean atSpeed = shooter.isAtGoalSpeed();
                  boolean hoodReady = hood.atTarget();
                  boolean looselyOnTarget = turret.isLooselyOnTarget();
                  boolean justSpinUp = turret.justSpinUp();
                  System.out.println(
                      "onTarget="
                          + onTarget
                          + " atSpeed="
                          + atSpeed
                          + " hoodReady="
                          + hoodReady
                          + " passing="
                          + passing
                          + " justSpinUp="
                          + justSpinUp
                          + " shiftActive="
                          + shiftActive);
                  if (override) {
                    shooter.setState(ShooterSubsystem.ShooterState.OVERRIDE);
                    //                    indexer.feed();
                  } else if (justSpinUp) {
                    shooter.setState(ShooterSubsystem.ShooterState.SPIN_UP);
                  } else if (passing && looselyOnTarget) {
                    shooter.setState(ShooterSubsystem.ShooterState.PASSING);
                  } else if (onTarget && atSpeed && hoodReady && r2Held && shiftActive) {
                    shooter.setState(ShooterSubsystem.ShooterState.RAPID_FIRE);
                    //                    indexer.feed();
                  } else {
                    shooter.setState(ShooterSubsystem.ShooterState.SPIN_UP);
                  }
                },
                shooter))
        .onFalse(
            Commands.runOnce(
                () -> {
                  shooter.setState(ShooterSubsystem.ShooterState.SPIN_UP);
                  //                    indexer.stop();
                },
                shooter,
                indexer));

    // L1: press to intake + feed indexer
    ps5Controller
        .L1()
        .and(ps5Controller.R1().negate())
        .onTrue(
            Commands.runOnce(
                () -> {
                  intake.intake();
                  //                  indexer.feed();
                },
                intake,
                indexer));

    // R1: press to stow intake + stop indexer
    ps5Controller
        .R1()
        .and(ps5Controller.L1().negate())
        .onTrue(
            Commands.runOnce(
                () -> {
                  intake.stow();
                  //                  indexer.stop();
                },
                intake,
                indexer));

    // L1 + R1 together: reverse handoff and indexer and intake
    ps5Controller
        .L1()
        .and(ps5Controller.R1())
        .whileTrue(
            Commands.runOnce(
                () -> {
                  handoff.setReverse(true);
                  indexer.reverse();
                  intake.eject();
                },
                handoff,
                indexer,
                intake))
        .onFalse(
            Commands.runOnce(
                () -> {
                  handoff.setReverse(false);
                  indexer.stop();
                  intake.stow();
                },
                handoff,
                indexer,
                intake));

    //    // D-pad up: chase nearest detected ball
    //    ps5Controller
    //        .povUp()
    //        .whileTrue(new ChaseBallCommand(drive, objectDetection, intake));

    // Shift change rumble alerts — 0.5s pulse at 10s and 5s remaining
    Trigger tenSecWarning =
        new Trigger(
            () -> {
              double remaining = HubShiftUtil.getOfficialShiftInfo().remainingTime();
              return remaining <= 10.0 && remaining > 9.5;
            });
    Trigger fiveSecWarning =
        new Trigger(
            () -> {
              double remaining = HubShiftUtil.getOfficialShiftInfo().remainingTime();
              return remaining <= 5.0 && remaining > 4.5;
            });

    tenSecWarning
        .whileTrue(
            Commands.run(() -> ps5Controller.getHID().setRumble(RumbleType.kBothRumble, 0.3)))
        .onFalse(
            Commands.runOnce(() -> ps5Controller.getHID().setRumble(RumbleType.kBothRumble, 0.0)));

    fiveSecWarning
        .whileTrue(
            Commands.run(() -> ps5Controller.getHID().setRumble(RumbleType.kBothRumble, 1.0)))
        .onFalse(
            Commands.runOnce(() -> ps5Controller.getHID().setRumble(RumbleType.kBothRumble, 0.0)));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  /** Call from Robot.robotPeriodic() to update Elastic dashboard. */
  public void updateDashboard() {
    // Field — robot pose + detected balls
    field.setRobotPose(drive.getPose());
    List<Translation2d> fuels = objectDetection.getFuelTranslations();
    Pose2d[] fuelPoses =
        fuels.stream().map(t -> new Pose2d(t, new Rotation2d())).toArray(Pose2d[]::new);
    field.getObject("Fuel Balls").setPoses(fuelPoses);

    // Handoff
    SmartDashboard.putBoolean("Handoff/Beam Break", handoff.isBeamBroken());
    SmartDashboard.putString("Handoff/State", handoff.getStateString());

    // Intake
    SmartDashboard.putString("Intake/State", intake.getState().name());
    SmartDashboard.putBoolean(
        "Intake/Deployed", intake.getState() != IntakeSubsystem.IntakeState.STOWED);

    // Turret — publish angle as a gyro-style Sendable for radial gauge in Elastic
    SmartDashboard.putBoolean("Turret/On Target", turret.isOnTarget());
    SmartDashboard.putBoolean("Turret/Passing Mode", turret.isPassingMode());
    SmartDashboard.putBoolean("Turret/At Goal", turret.atGoal());
    SmartDashboard.putNumber("Turret/Angle", turret.getAbsolutePositionDeg());

    // Hood
    SmartDashboard.putBoolean("Hood/At Target", hood.atTarget());
    SmartDashboard.putNumber("Hood/Position", hood.getPosition());

    // Shooter
    SmartDashboard.putString("Shooter/State", shooter.getState().name());
    SmartDashboard.putBoolean("Shooter/At Speed", shooter.isAtGoalSpeed());

    // Hub shift timing
    HubShiftUtil.ShiftInfo shiftInfo = HubShiftUtil.getOfficialShiftInfo();
    SmartDashboard.putString("Match/Current Shift", shiftInfo.currentShift().name());
    SmartDashboard.putNumber("Match/Shift Time Left", shiftInfo.remainingTime());
    SmartDashboard.putNumber(
        "Match/Match Time", edu.wpi.first.wpilibj.DriverStation.getMatchTime());

    // Indexer
    SmartDashboard.putString("Indexer/State", indexer.getState().name());

    // Object detection
    SmartDashboard.putNumber("Detection/Ball Count", objectDetection.getTrackedFuelCount());

    // Driver camera view
    driverView.update();

    // PDH / current draw logging (every 10 loops ~200ms to reduce CAN load)
    if (++pdhLogCounter >= 10) {
      pdhLogCounter = 0;
      Logger.recordOutput("PDH/Voltage", pdh.getVoltage());
      Logger.recordOutput("PDH/TotalCurrent", pdh.getTotalCurrent());
      for (int ch = 0; ch < 24; ch++) {
        Logger.recordOutput("PDH/Channel" + ch, pdh.getCurrent(ch));
      }
    }
  }

  /** Resets the pose to the alliance zone if no auto was run (pose is still near origin). */
  public void resetPoseForAlliance() {
    Pose2d currentPose = drive.getPose();
    // Only reset if pose is still near the default (0,0) — meaning no auto ran
    if (currentPose.getTranslation().getNorm() < 1.0) {
      Pose2d startingPose;
      if (AllianceFlipUtil.shouldFlip()) {
        // Red alliance: place robot in red zone facing blue wall
        startingPose =
            new Pose2d(
                FieldConstants.FIELDLENGTH - 1.0, FieldConstants.FIELDWIDTH / 2.0, Rotation2d.kPi);
      } else {
        // Blue alliance: place robot in blue zone facing red wall
        startingPose = new Pose2d(1.0, FieldConstants.FIELDWIDTH / 2.0, Rotation2d.kZero);
      }
      drive.setPose(startingPose);
    }
  }

  // returns from 0.3 to 1, depending on how close we are to hub, if we close then 0.3, if we far
  // then 1
  private double getShootingSpeedFactor() {
    boolean shooting = ps5Controller.R2().getAsBoolean() || ps5Controller.L2().getAsBoolean();
    if (!shooting) return 1.0;

    Pose2d pose = drive.getPose();
    double hubDist;
    if (AllianceFlipUtil.shouldFlip()) {
      // red aliacn so hub is near x = FIELDLENGTH - ALLIANCEWALLTOHUB
      double hubX = FieldConstants.FIELDLENGTH - FieldConstants.ALLIANCEWALLTOHUB;
      if (pose.getX() < hubX) return 1.0; // not in alliance zone
      hubDist = pose.getX() - hubX;
    } else {
      // blue hub is near x = ALLIANCEWALLTOHUB
      double hubX = FieldConstants.ALLIANCEWALLTOHUB;
      if (pose.getX() > hubX) return 1.0; // not in alliance zone
      hubDist = hubX - pose.getX();
    }

    double minSpeed = 0.3; // tune: speed when right at hub
    double maxDist = FieldConstants.ALLIANCEWALLTOHUB; // tune: distance at which full speed
    double factor = minSpeed + (1.0 - minSpeed) * Math.min(hubDist / maxDist, 1.0);
    return factor;
  }
}
