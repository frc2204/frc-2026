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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
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
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.AdaptiveAutoCommand;
import frc.robot.commands.ChaseBallCommand;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.ShootingCommands;
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
import java.util.function.DoubleSupplier;
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

  // Controllers
  private final CommandXboxController driverController = new CommandXboxController(0);
  private final CommandPS5Controller operatorController = new CommandPS5Controller(1);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;
  private final Field2d field = new Field2d();
  private int pdhLogCounter = 0;
  private boolean slowMode = false;
  private boolean overrideSpeed = false;
  private boolean intakeDeployed = false;
  private double lastIntakeToggleTime = 0;
  private boolean intakeDriveEnabled = false;
  private Rotation2d intakeHeading = Rotation2d.kZero;

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
                () -> drive.getChassisSpeeds().omegaRadiansPerSecond,
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
                () -> drive.getChassisSpeeds().omegaRadiansPerSecond,
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

        vision =
            new Vision(
                drive::addVisionMeasurement,
                () -> drive.getChassisSpeeds().omegaRadiansPerSecond,
                new VisionIO() {},
                new VisionIO() {});
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
        "spinUpShooter", ShootingCommands.spinUp(shooter));
    com.pathplanner.lib.auto.NamedCommands.registerCommand(
        "idleShooter", ShootingCommands.idle(shooter));
    com.pathplanner.lib.auto.NamedCommands.registerCommand(
        "overrideShooter",
        Commands.runOnce(() -> shooter.setState(ShooterSubsystem.ShooterState.OVERRIDE), shooter));
    com.pathplanner.lib.auto.NamedCommands.registerCommand(
        "autoShoot", ShootingCommands.autoShoot(turret, shooter, hood, drive::getPose));

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
    // Default command, field-relative drive with speed modes
    // PS button toggles intake drive mode (heading follows stick direction)
    double slowFactor = 0.65; // tune
    var xSup =
        (DoubleSupplier) () -> -driverController.getLeftY() * getDriveSpeedFactor(slowFactor);
    var ySup =
        (DoubleSupplier) () -> -driverController.getLeftX() * getDriveSpeedFactor(slowFactor);
    var omegaSup =
        (DoubleSupplier) () -> -driverController.getRightX() * getDriveSpeedFactor(slowFactor);

    driverController
        .povUp()
        .onTrue(
            Commands.sequence(
                Commands.runOnce(
                    () -> {
                      intakeDriveEnabled = !intakeDriveEnabled;
                      if (intakeDriveEnabled) {
                        intakeHeading = drive.getRotation();
                      }
                    }),
                Commands.runOnce(() -> {}, drive)));

    drive.setDefaultCommand(
        Commands.either(
            DriveCommands.joystickDriveAtAngle(
                drive,
                xSup,
                ySup,
                () -> {
                  double x = -driverController.getLeftY();
                  double y = -driverController.getLeftX();
                  if (Math.hypot(x, y) > 0.1) {
                    double heading = Math.atan2(y, x);
                    boolean isFlipped =
                        DriverStation.getAlliance().isPresent()
                            && DriverStation.getAlliance().get() == Alliance.Red;
                    if (isFlipped) heading += Math.PI;
                    intakeHeading = Rotation2d.fromRadians(heading);
                  }
                  return intakeHeading;
                }),
            DriveCommands.joystickDrive(drive, xSup, ySup, omegaSup),
            () -> intakeDriveEnabled));

    // X: toggle override speed (full speed, ignores shooting slowdown)
    driverController.back().onTrue(Commands.runOnce(() -> overrideSpeed = !overrideSpeed));

    // TODO:  make triangle spin up and down

    // R3: snap drivetrain heading to nearest 0° or 180° (intake faces forward/backward)
    driverController
        .povDown()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> -driverController.getLeftY() * getDriveSpeedFactor(slowFactor),
                () -> -driverController.getLeftX() * getDriveSpeedFactor(slowFactor),
                () -> {
                  double heading = drive.getRotation().getRadians();
                  // Normalize to [-PI, PI] then snap to 0 or PI
                  heading = Math.IEEEremainder(heading, 2.0 * Math.PI);
                  if (heading > Math.PI / 2.0) return Rotation2d.kPi;
                  if (heading < -Math.PI / 2.0) return Rotation2d.kPi;
                  return Rotation2d.kZero;
                }));

    // A: toggle slow mode
    driverController.start().onTrue(Commands.runOnce(() -> slowMode = !slowMode));

    // L3: reset gyro to 0°
    driverController
        .leftStick()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), Rotation2d.kZero)),
                    drive)
                .ignoringDisable(true));

    // Y: spin up shooter
    driverController
        .y()
        .onTrue(
            Commands.runOnce(
                () -> shooter.setState(ShooterSubsystem.ShooterState.SPIN_UP), shooter));

    // B: idle shooter
    driverController
        .b()
        .onTrue(
            Commands.runOnce(() -> shooter.setState(ShooterSubsystem.ShooterState.IDLE), shooter));

    // R2 or L2: either trigger activates shooting system
    // Both together: force fire override
    driverController
        .rightTrigger()
        .or(driverController.leftTrigger())
        .whileTrue(
            ShootingCommands.shootWhileHeld(
                turret,
                shooter,
                hood,
                drive::getPose,
                () ->
                    driverController.rightTrigger().getAsBoolean()
                        && driverController.leftTrigger().getAsBoolean(),
                driverController.rightTrigger()::getAsBoolean))
        .onFalse(ShootingCommands.spinUp(shooter));

    // LB: toggle intake deploy/stow (debounced)
    driverController // later ask if daniel wants it to be like hold for 1 sec and stop spinning
        // intake
        // but its still down
        .leftBumper()
        .onTrue(
            Commands.runOnce(
                () -> {
                  double now = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
                  if (now - lastIntakeToggleTime < 0.3) return; // debounce
                  lastIntakeToggleTime = now;
                  intakeDeployed = !intakeDeployed;
                  if (intakeDeployed) {
                    intake.intake();
                  } else {
                    intake.stow();
                  }
                },
                intake));

    // RB: override — reverse handoff, indexer, intake while held
    driverController
        .rightBumper()
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
                  if (intakeDeployed) {
                    intake.intake();
                  } else {
                    intake.stow();
                  }
                },
                handoff,
                indexer,
                intake));

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
            Commands.run(() -> driverController.getHID().setRumble(RumbleType.kBothRumble, 0.3)))
        .onFalse(
            Commands.runOnce(
                () -> driverController.getHID().setRumble(RumbleType.kBothRumble, 0.0)));

    fiveSecWarning
        .whileTrue(
            Commands.run(() -> driverController.getHID().setRumble(RumbleType.kBothRumble, 1.0)))
        .onFalse(
            Commands.runOnce(
                () -> driverController.getHID().setRumble(RumbleType.kBothRumble, 0.0)));

    // ── PS5 CONTROLLER (port 1) — operator live tuning ──────────────────
    // Triangle: +25 RPM at nearest distance point
    operatorController
        .triangle()
        .onTrue(
            Commands.runOnce(
                () -> shooter.adjustRpmTrimAtDistance(shooter.getTargetDistance(), 25.0), shooter));

    // Cross: -25 RPM at nearest distance point
    operatorController
        .cross()
        .onTrue(
            Commands.runOnce(
                () -> shooter.adjustRpmTrimAtDistance(shooter.getTargetDistance(), -25.0),
                shooter));

    // R2: +25 RPM global (all distances)
    operatorController
        .R2()
        .onTrue(Commands.runOnce(() -> shooter.adjustGlobalRpmTrim(25.0), shooter));

    // L2: -25 RPM global (all distances)
    operatorController
        .L2()
        .onTrue(Commands.runOnce(() -> shooter.adjustGlobalRpmTrim(-25.0), shooter));

    // POV Up: hood +0.014 rotations (~5°) at nearest distance point
    operatorController
        .povUp()
        .onTrue(
            Commands.runOnce(
                () -> hood.adjustHoodTrimAtDistance(hood.getTargetDistance(), 0.014), hood));

    // POV Down: hood -0.014 rotations (~5°) at nearest distance point
    operatorController
        .povDown()
        .onTrue(
            Commands.runOnce(
                () -> hood.adjustHoodTrimAtDistance(hood.getTargetDistance(), -0.014), hood));

    // R1: hood +0.014 rotations global
    operatorController.R1().onTrue(Commands.runOnce(() -> hood.adjustGlobalHoodTrim(0.014), hood));

    // L1: hood -0.014 rotations global
    operatorController.L1().onTrue(Commands.runOnce(() -> hood.adjustGlobalHoodTrim(-0.014), hood));

    // Left Stick X: turret offset ±17° (position-based, centered = 0)
    new Trigger(() -> Math.abs(operatorController.getLeftX()) > 0.1)
        .whileTrue(
            Commands.run(
                () -> turret.setTurretManualOffset(operatorController.getLeftX() * 17.0), turret))
        .onFalse(Commands.runOnce(() -> turret.resetTurretManualOffset(), turret));

    // Right Stick Y: manual intake deploy position control
    new Trigger(() -> Math.abs(operatorController.getRightY()) > 0.1)
        .whileTrue(
            Commands.run(
                () -> {
                  intake.setState(IntakeSubsystem.IntakeState.MANUAL);
                  intake.setManualVoltage(-operatorController.getRightY() * 4.0); // tune voltage
                },
                intake))
        .onFalse(
            Commands.runOnce(
                () -> {
                  intake.setManualVoltage(0.0);
                  intake.stow();
                },
                intake));

    // PS: reset all trims
    operatorController
        .PS()
        .onTrue(
            Commands.runOnce(
                () -> {
                  shooter.resetRpmTrim();
                  hood.resetHoodTrim();
                  turret.resetTurretManualOffset();
                },
                shooter,
                hood,
                turret));
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
    SmartDashboard.putNumber("Shooter/GlobalRpmTrim", shooter.getGlobalRpmTrim());

    // Hood trim
    SmartDashboard.putNumber("Hood/GlobalHoodTrim", hood.getGlobalHoodTrim());

    // Turret trim
    SmartDashboard.putNumber("Turret/ManualOffset", turret.getTurretManualOffset());

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

  public Drive getDrive() {
    return drive;
  }

  public TurretSubsystem getTurret() {
    return turret;
  }

  private double getDriveSpeedFactor(double slowFactor) {
    if (overrideSpeed) return 1.0;
    if (slowMode) return slowFactor;
    return getShootingSpeedFactor();
  }

  // right now try with just a constant speed like slowfactor untill shooting on the move is
  // better??

  // returns from 0.3 to 0.8 when in alliance zone (shooting at hub), 1.0 otherwise
  private double getShootingSpeedFactor() {
    boolean shooting =
        driverController.rightTrigger().getAsBoolean()
            || driverController.leftTrigger().getAsBoolean();
    if (!shooting) return 1.0;

    Pose2d pose = drive.getPose();
    double hubDist;
    if (AllianceFlipUtil.shouldFlip()) {
      // red alliance so hub is near x = FIELDLENGTH - ALLIANCEWALLTOHUB
      double hubX = FieldConstants.FIELDLENGTH - FieldConstants.ALLIANCEWALLTOHUB;
      if (pose.getX() < hubX) return 1.0; // not in alliance zone, full speed
      hubDist = pose.getX() - hubX;
    } else {
      // blue hub is near x = ALLIANCEWALLTOHUB
      double hubX = FieldConstants.ALLIANCEWALLTOHUB;
      if (pose.getX() > hubX) return 1.0; // not in alliance zone, full speed
      hubDist = hubX - pose.getX();
    }

    double minSpeed = 0.3; // tune: speed when right at hub
    double maxSpeed = 0.8; // tune: cap speed even at edge of alliance zone
    double maxDist = FieldConstants.ALLIANCEWALLTOHUB;
    double factor = minSpeed + (maxSpeed - minSpeed) * Math.min(hubDist / maxDist, 1.0);
    return factor;
  }
}
