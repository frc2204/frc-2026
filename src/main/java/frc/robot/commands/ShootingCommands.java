package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.shooting.HoodSubsystem;
import frc.robot.subsystems.shooting.ShooterSubsystem;
import frc.robot.subsystems.shooting.ShooterSubsystem.ShooterState;
import frc.robot.subsystems.shooting.TurretSubsystem;
import frc.robot.util.FieldConstants;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class ShootingCommands {

  /**
   * Returns a command that runs the full shooting state machine each cycle. Used by both teleop
   * triggers and autonomous named commands.
   *
   * @param turret turret subsystem
   * @param shooter shooter subsystem
   * @param hood hood subsystem
   * @param poseSupplier robot pose supplier
   * @param overrideSupplier true when force-fire override is active
   * @param r2Supplier true when the primary fire trigger (R2) is held
   */
  public static Command shootWhileHeld(
      TurretSubsystem turret,
      ShooterSubsystem shooter,
      HoodSubsystem hood,
      Supplier<Pose2d> poseSupplier,
      BooleanSupplier overrideSupplier,
      BooleanSupplier r2Supplier) {
    return Commands.run(
        () -> {
          boolean override = overrideSupplier.getAsBoolean();
          boolean r2Held = r2Supplier.getAsBoolean();
          boolean insideTower = FieldConstants.isInsideTower(poseSupplier.get());
          boolean trenchMode = hood.isTrenchMode();
          boolean onTarget = turret.isOnTarget();
          boolean passing = turret.isPassingMode();
          boolean atSpeed = shooter.isAtGoalSpeed();
          boolean hoodReady = hood.atTarget();
          boolean looselyOnTarget = turret.isLooselyOnTarget();
          boolean justSpinUp = turret.justSpinUp();

          Logger.recordOutput("Shooting/onTarget", onTarget);
          Logger.recordOutput("Shooting/atSpeed", atSpeed);
          Logger.recordOutput("Shooting/hoodReady", hoodReady);
          Logger.recordOutput("Shooting/passing", passing);
          Logger.recordOutput("Shooting/justSpinUp", justSpinUp);
          Logger.recordOutput("Shooting/trenchMode", trenchMode);

          if (override) {
            shooter.setState(ShooterState.OVERRIDE);
          } else if (trenchMode || insideTower) {
            shooter.setState(ShooterState.SPIN_UP);
          } else if (justSpinUp) {
            shooter.setState(ShooterState.SPIN_UP);
          } else if (passing && looselyOnTarget) {
            shooter.setState(ShooterState.PASSING);
          } else if (onTarget && atSpeed && hoodReady && r2Held
          /* && shiftActive */ ) {
            shooter.setState(ShooterState.RAPID_FIRE);
          } else {
            shooter.setState(ShooterState.SPIN_UP);
          }
        },
        shooter);
  }

  /**
   * Returns a command for autonomous that spins up and fires when ready. No trigger gating — fires
   * as soon as all conditions are met.
   */
  public static Command autoShoot(
      TurretSubsystem turret,
      ShooterSubsystem shooter,
      HoodSubsystem hood,
      Supplier<Pose2d> poseSupplier) {
    return shootWhileHeld(turret, shooter, hood, poseSupplier, () -> false, () -> true);
  }

  /** Spin up without firing. */
  public static Command spinUp(ShooterSubsystem shooter) {
    return Commands.runOnce(() -> shooter.setState(ShooterState.SPIN_UP), shooter);
  }

  /** Idle the shooter. */
  public static Command idle(ShooterSubsystem shooter) {
    return Commands.runOnce(() -> shooter.setState(ShooterState.IDLE), shooter);
  }
}
