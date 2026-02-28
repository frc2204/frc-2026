package frc.robot.subsystems.handoff;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.intake.IndexerSubsystem;
import frc.robot.subsystems.shooting.ShooterSubsystem;
import frc.robot.subsystems.shooting.ShooterSubsystem.ShooterState;

public class HandoffSubsystem extends SubsystemBase {

  private static final HandoffSubsystem INSTANCE = new HandoffSubsystem();

  public static HandoffSubsystem getInstance() {
    return INSTANCE;
  }

  private static final int HANDOFF_MOTOR_ID = 41;
  private static final int BEAM_BREAK_DIO = 0;

  private static final double FEED_VOLTAGE = -8.0; // tune
  private static final double REVERSE_VOLTAGE = 4.0; // tune

  // if beam break is blocked for this long reverse
  private static final double JAM_TIME_SECONDS = 0.5; // tune
  private static final double UNJAM_TIME_SECONDS = 0.3; // tune

  private final SparkMax handoffMotor = new SparkMax(HANDOFF_MOTOR_ID, MotorType.kBrushless);
  private final DigitalInput beamBreak = new DigitalInput(BEAM_BREAK_DIO);
  private boolean forceReverse = false;

  private double blockedStartTime = -1.0;
  private double unjamStartTime = -1.0;
  private boolean unjamming = false;

  private HandoffSubsystem() {
    SparkMaxConfig config = new SparkMaxConfig();
    config.idleMode(IdleMode.kCoast).smartCurrentLimit(30).inverted(false);
    handoffMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
    ShooterSubsystem.ShooterState st = ShooterSubsystem.getInstance().getState();
    //    System.out.println(
    //        "handoff: beamBroken="
    //            + isBeamBroken()
    //            + " shooterState="
    //            + st
    //            + " unjamming="
    //            + unjamming);
    if (forceReverse) {
      handoffMotor.setVoltage(REVERSE_VOLTAGE);
      resetJamState();
      return;
    }

    ShooterSubsystem shooter = ShooterSubsystem.getInstance();
    ShooterState shooterState = shooter.getState();

    boolean shouldFeed = false;
    if (shooterState == ShooterState.PASSING) {
      shouldFeed = ShooterSubsystem.getInstance().isAtGoalSpeed(); // full voltage, feed
    }
    //    shooterState == ShooterState.PASSING ||
    //    if (shooterState == ShooterState.OVERRIDE) {
    //      shouldFeed = shooter.isAtGoalSpeed(); // full voltage, always feed
    //    } else
    if (shooterState == ShooterState.RAPID_FIRE) {
      shouldFeed = shooter.isAtGoalSpeed();
    } else if (shooterState == ShooterState.RAPID_FIRE_ACCURATE) {
      shouldFeed = shooter.isAtGoalSpeedAccurate();
    } else if (shooterState == ShooterState.OVERRIDE) {
      shouldFeed = true;
    }
    if (shouldFeed) {
      IndexerSubsystem.getInstance().feed();
    }

    if (!shouldFeed) {
      handoffMotor.setVoltage(0.0);
      IndexerSubsystem.getInstance().stop();
      resetJamState();
      return;
    }

    double now = Timer.getFPGATimestamp();

    if (unjamming) {
      handoffMotor.setVoltage(REVERSE_VOLTAGE);
      IndexerSubsystem.getInstance().reverse();
      if (now - unjamStartTime >= UNJAM_TIME_SECONDS) {
        unjamming = false;
        blockedStartTime = -1.0;
      }
      return;
    }

    boolean blocked = !beamBreak.get();

    if (blocked) {
      if (blockedStartTime < 0) {
        blockedStartTime = now;
      } else if (now - blockedStartTime >= JAM_TIME_SECONDS) {
        unjamming = true;
        unjamStartTime = now;
        return;
      }
    } else {
      blockedStartTime = -1.0;
    }

    handoffMotor.setVoltage(FEED_VOLTAGE);
  }

  private void resetJamState() {
    blockedStartTime = -1.0;
    unjamming = false;
  }

  public boolean isBeamBroken() {
    return !beamBreak.get();
  }

  public void setReverse(boolean reverse) {
    forceReverse = reverse;
  }

  public String getStateString() {
    if (forceReverse) return "REVERSING";
    if (unjamming) return "UNJAMMING";
    ShooterState shooterState = ShooterSubsystem.getInstance().getState();
    if (shooterState == ShooterState.PASSING) return "FEEDING";
    if (shooterState == ShooterState.RAPID_FIRE && ShooterSubsystem.getInstance().isAtGoalSpeed())
      return "FEEDING";
    if (shooterState == ShooterState.RAPID_FIRE_ACCURATE
        && ShooterSubsystem.getInstance().isAtGoalSpeedAccurate()) return "FEEDING";
    if (shooterState == ShooterState.OVERRIDE) return "FEEDING";
    return "IDLE";
  }
}
