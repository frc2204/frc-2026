// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import static frc.robot.subsystems.vision.VisionConstants.camera0Name;
import static frc.robot.subsystems.vision.VisionConstants.robotToCamera0;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.*;
import frc.robot.subsystems.handoff.HandoffSubsystem;
import frc.robot.subsystems.intake.IndexerSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.objectdetection.ObjectDetection;
import frc.robot.subsystems.shooting.ShooterSubsystem;
import frc.robot.subsystems.shooting.turrettestingSubsystem;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOLimelight;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
import frc.robot.util.HubShiftUtil;
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
  private final turrettestingSubsystem turret;
  private final ObjectDetection objectDetection;

  // Controller
  private final CommandPS5Controller controller = new CommandPS5Controller(0);
  private final CommandPS5Controller ps5Controller = new CommandPS5Controller(1);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

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
                new VisionIOLimelight(camera0Name, drive::getRotation));
        turret = new turrettestingSubsystem(drive::getPose, drive::getChassisSpeeds);
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
        turret = new turrettestingSubsystem(drive::getPose, drive::getChassisSpeeds);
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
        turret = new turrettestingSubsystem(drive::getPose, drive::getChassisSpeeds);
        break;
    }

    // Set up object detection
    objectDetection = new ObjectDetection(drive::getPose);

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
    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -controller.getLeftY(),
            () -> -controller.getLeftX(),
            () -> -controller.getRightX()));

    // Cross: X pattern wheel lock
    controller.cross().onTrue(Commands.runOnce(drive::stopWithX, drive));

    // Options: reset gyro to 0°
    controller
        .options()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), Rotation2d.kZero)),
                    drive)
                .ignoringDisable(true));

    // Triangle: spin up shooter
    controller
        .triangle()
        .onTrue(
            Commands.runOnce(
                () -> shooter.setState(ShooterSubsystem.ShooterState.SPIN_UP), shooter));

    // Circle: idle shooter
    controller
        .circle()
        .onTrue(
            Commands.runOnce(() -> shooter.setState(ShooterSubsystem.ShooterState.IDLE), shooter));

    // R2 or L2: either trigger activates shooting system
    // Both together: force fire override
    controller
        .R2()
        .or(controller.L2())
        .whileTrue(
            Commands.run(
                () -> {
                  boolean r2Held = controller.R2().getAsBoolean();
                  boolean l2Held = controller.L2().getAsBoolean();
                  boolean override = r2Held && l2Held;
                  boolean onTarget = turret.isOnTarget();
                  boolean passing = turret.isPassingMode();
                  boolean atSpeed = shooter.isAtGoalSpeed();
                  boolean looselyOnTarget = turret.isLooselyOnTarget();

                  if (override) {
                    shooter.setState(ShooterSubsystem.ShooterState.RAPID_FIRE);
                  } else if (passing && looselyOnTarget) {
                    shooter.setState(ShooterSubsystem.ShooterState.PASSING);
                  } else if (onTarget && atSpeed) {
                    shooter.setState(ShooterSubsystem.ShooterState.RAPID_FIRE);
                  } else {
                    shooter.setState(ShooterSubsystem.ShooterState.SPIN_UP);
                  }
                },
                shooter))
        .onFalse(
            Commands.runOnce(
                () -> shooter.setState(ShooterSubsystem.ShooterState.SPIN_UP), shooter));

    // L1: press to intake + feed indexer
    controller
        .L1()
        .and(controller.R1().negate())
        .onTrue(
            Commands.runOnce(
                () -> {
                  intake.intake();
                  indexer.feed();
                }));

    // R1: press to stow intake + stop indexer
    controller
        .R1()
        .and(controller.L1().negate())
        .onTrue(
            Commands.runOnce(
                () -> {
                  intake.stow();
                  indexer.stop();
                }));

    // L1 + R1 together: reverse handoff and indexer
    controller
        .L1()
        .and(controller.R1())
        .whileTrue(
            Commands.runOnce(
                () -> {
                  handoff.setReverse(true);
                  indexer.reverse();
                }))
        .onFalse(
            Commands.runOnce(
                () -> {
                  handoff.setReverse(false);
                  indexer.stop();
                }));

    // Shift change rumble alerts
    Trigger tenSecWarning =
        new Trigger(
            () -> {
              double remaining = HubShiftUtil.getShiftedShiftInfo().remainingTime();
              return remaining <= 10.0 && remaining > 5.0;
            });
    Trigger fiveSecWarning =
        new Trigger(
            () -> {
              double remaining = HubShiftUtil.getShiftedShiftInfo().remainingTime();
              return remaining <= 5.0 && remaining > 0.0;
            });

    tenSecWarning
        .whileTrue(Commands.run(() -> controller.getHID().setRumble(RumbleType.kBothRumble, 0.3)))
        .onFalse(
            Commands.runOnce(() -> controller.getHID().setRumble(RumbleType.kBothRumble, 0.0)));

    fiveSecWarning
        .whileTrue(Commands.run(() -> controller.getHID().setRumble(RumbleType.kBothRumble, 1.0)))
        .onFalse(
            Commands.runOnce(() -> controller.getHID().setRumble(RumbleType.kBothRumble, 0.0)));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
