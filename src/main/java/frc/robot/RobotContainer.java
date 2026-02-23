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
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.*;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooting.HoodSubsystem;
import frc.robot.subsystems.shooting.ShooterSubsystem;
import frc.robot.subsystems.shooting.TurretSubsystem;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOLimelight;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * Tuning branch RobotContainer. Swerve drive + shooter, hood, turret, and intake for finding
 * interpolation map data points.
 *
 * <p>Workflow: drive to a distance, spin up shooter, adjust hood via dashboard, fire, record
 * distance/RPM/hood as a data point.
 */
public class RobotContainer {
  // Subsystems
  private final Drive drive;
  private final Vision vision;
  private final ShooterSubsystem shooter = ShooterSubsystem.getInstance();
  private final HoodSubsystem hood = HoodSubsystem.getInstance();
  private final IntakeSubsystem intake = IntakeSubsystem.getInstance();
  private final TurretSubsystem turret;

  // Controller
  private final CommandPS5Controller controller = new CommandPS5Controller(0);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
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
        turret = new TurretSubsystem(drive::getPose, drive::getChassisSpeeds);
        break;

      case SIM:
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

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());
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

    // Elastic dashboard: swerve widget
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

    // Tuning dashboard defaults
    SmartDashboard.putNumber("Tuning/Target Hood Position", 0.1);
    SmartDashboard.putNumber("Tuning/Target Shooter RPM", 4200.0);

    configureButtonBindings();
  }

  /**
   * Button mappings for tuning:
   *
   * <ul>
   *   <li>Left stick: drive
   *   <li>Right stick: rotate
   *   <li>Triangle: spin up shooter
   *   <li>Circle: stop shooter
   *   <li>Cross: slow drive (0.25x)
   *   <li>Options: reset gyro
   *   <li>L1: intake
   *   <li>R1: stow intake
   *   <li>D-pad up/down: bump hood position +/- 0.01
   * </ul>
   */
  private void configureButtonBindings() {
    // Default command: field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -controller.getLeftY(),
            () -> -controller.getLeftX(),
            () -> -controller.getRightX()));

    // Cross: slow drive
    double slowFactor = 0.25;
    controller
        .cross()
        .whileTrue(
            DriveCommands.joystickDrive(
                drive,
                () -> -controller.getLeftY() * slowFactor,
                () -> -controller.getLeftX() * slowFactor,
                () -> -controller.getRightX() * slowFactor));

    // Options: reset gyro
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

    // R2: rapid fire (force feed)
    controller
        .R2()
        .whileTrue(
            Commands.run(() -> shooter.setState(ShooterSubsystem.ShooterState.RAPID_FIRE), shooter))
        .onFalse(
            Commands.runOnce(
                () -> shooter.setState(ShooterSubsystem.ShooterState.SPIN_UP), shooter));

    // L1: intake
    controller.L1().onTrue(Commands.runOnce(() -> intake.intake()));

    // R1: stow intake
    controller.R1().onTrue(Commands.runOnce(() -> intake.stow()));

    // D-pad up: bump hood position +0.01
    controller
        .povUp()
        .onTrue(
            Commands.runOnce(
                () -> {
                  double current = SmartDashboard.getNumber("Tuning/Target Hood Position", 0.1);
                  SmartDashboard.putNumber("Tuning/Target Hood Position", current + 0.01);
                }));

    // D-pad down: bump hood position -0.01
    controller
        .povDown()
        .onTrue(
            Commands.runOnce(
                () -> {
                  double current = SmartDashboard.getNumber("Tuning/Target Hood Position", 0.1);
                  SmartDashboard.putNumber("Tuning/Target Hood Position", current - 0.01);
                }));

    // D-pad right: bump shooter RPM +50
    controller
        .povRight()
        .onTrue(
            Commands.runOnce(
                () -> {
                  double current = SmartDashboard.getNumber("Tuning/Target Shooter RPM", 4200.0);
                  SmartDashboard.putNumber("Tuning/Target Shooter RPM", current + 50.0);
                }));

    // D-pad left: bump shooter RPM -50
    controller
        .povLeft()
        .onTrue(
            Commands.runOnce(
                () -> {
                  double current = SmartDashboard.getNumber("Tuning/Target Shooter RPM", 4200.0);
                  SmartDashboard.putNumber("Tuning/Target Shooter RPM", current - 50.0);
                }));
  }

  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  /** Call from Robot.robotPeriodic() to update tuning dashboard. */
  public void updateTuningDashboard() {
    // Override hood from dashboard (bypass distance-based auto)
    double targetHoodPos = SmartDashboard.getNumber("Tuning/Target Hood Position", 0.1);
    hood.setPosition(targetHoodPos);

    // Override shooter RPM from dashboard (bypass distance-based map)
    double targetRPM = SmartDashboard.getNumber("Tuning/Target Shooter RPM", 4200.0);
    shooter.setTargetRPMOverride(targetRPM);

    // Distance to hub
    double distanceToHub = turret.getDistanceFromHub(drive.getPose());

    // Outputs
    SmartDashboard.putNumber("Tuning/Distance To Hub", distanceToHub);
    SmartDashboard.putNumber("Tuning/Current Shooter RPM", shooter.getVelocityRPM());
    SmartDashboard.putNumber("Tuning/Current Hood Position", hood.getPosition());
    SmartDashboard.putNumber("Tuning/Turret Angle", turret.getAbsolutePositionDeg());
    SmartDashboard.putString("Tuning/Shooter State", shooter.getState().name());
    SmartDashboard.putBoolean("Tuning/Shooter At Speed", shooter.isAtGoalSpeed());
    SmartDashboard.putBoolean("Tuning/Hood At Target", hood.atTarget());

    // Intake
    SmartDashboard.putString("Tuning/Intake State", intake.getState().name());

    // Robot pose
    SmartDashboard.putNumber("Tuning/Robot X", drive.getPose().getX());
    SmartDashboard.putNumber("Tuning/Robot Y", drive.getPose().getY());
  }
}
