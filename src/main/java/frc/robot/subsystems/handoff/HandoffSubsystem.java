package frc.robot.subsystems.handoff;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.intake.IndexerSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooting.ShooterSubsystem;
import frc.robot.subsystems.shooting.ShooterSubsystem.ShooterState;
import org.littletonrobotics.junction.Logger;

public class HandoffSubsystem extends SubsystemBase {

  private static final HandoffSubsystem INSTANCE = new HandoffSubsystem();

  public static HandoffSubsystem getInstance() {
    return INSTANCE;
  }

  private static final int HANDOFF_MOTOR_ID = 41;
  private static final int BEAM_BREAK_DIO = 0;

  private static final double FEED_RPS = -100.0; // tune — feed speed in RPS
  private static final double REVERSE_RPS = 30.0; // tune — unjam/reverse speed in RPS
  private static final double STAGE_RPS = -30.0; // tune — slow feed to stage ball at beam break

  private static final double HANDOFF_KS = 3.0; // amps — overcome friction, tune
  private static final double HANDOFF_KP = 3.0; // amps per RPS error, tune

  // if beam break is blocked for this long reverse
  private static final double JAM_TIME_SECONDS = 0.5; // tune
  private static final double UNJAM_TIME_SECONDS = 0.3; // tune
  private static final double FEED_LATCH_TIME =
      0.5; // seconds — min time to keep feeding. 0 = no latch

  private final TalonFX handoffMotor = new TalonFX(HANDOFF_MOTOR_ID);
  private final VelocityTorqueCurrentFOC velocityRequest = new VelocityTorqueCurrentFOC(0);
  private final NeutralOut neutralRequest = new NeutralOut();
  private final DigitalInput beamBreak = new DigitalInput(BEAM_BREAK_DIO);
  private boolean forceReverse = false;

  private double feedLatchStartTime = -1.0;
  private double blockedStartTime = -1.0;
  private double unjamStartTime = -1.0;
  private boolean unjamming = false;

  private boolean lastBeamBroken = false;
  private int ballsScored = 0;

  private HandoffSubsystem() {
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.StatorCurrentLimit = 120;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = 40; // peak spike
    config.CurrentLimits.SupplyCurrentLowerLimit = 25; // steady state after spike
    config.CurrentLimits.SupplyCurrentLowerTime = 3.0; // seconds at peak before dropping
    config.Slot0.kS = HANDOFF_KS;
    config.Slot0.kP = HANDOFF_KP;
    handoffMotor.getConfigurator().apply(config);
  }

  @Override
  public void periodic() {
    boolean currentBeamBroken = isBeamBroken();
    if (currentBeamBroken && !lastBeamBroken) {
      ballsScored++;
    }
    lastBeamBroken = currentBeamBroken;
    Logger.recordOutput("Handoff/BallsScored", ballsScored);
    Logger.recordOutput("Handoff/BeamBroken", currentBeamBroken);
    Logger.recordOutput("Handoff/BeamRaw", beamBreak.get());

    ShooterSubsystem.ShooterState st = ShooterSubsystem.getInstance().getState();
    //    System.out.println(
    //        "handoff: beamBroken="
    //            + isBeamBroken()
    //            + " shooterState="
    //            + st
    //            + " unjamming="
    //            + unjamming);
    if (forceReverse) {
      handoffMotor.setControl(velocityRequest.withVelocity(REVERSE_RPS));
      resetJamState();
      return;
    }

    // Stage ball: while intaking and beam break not yet tripped, slowly run handoff
    // to pull ball forward and stage it, freeing space in the intake for another ball
    IntakeSubsystem.IntakeState intakeState = IntakeSubsystem.getInstance().getState();
    boolean intaking =
        intakeState == IntakeSubsystem.IntakeState.INTAKING
            || intakeState == IntakeSubsystem.IntakeState.DEPLOYING;

    ShooterSubsystem shooter = ShooterSubsystem.getInstance();
    ShooterState shooterState = shooter.getState();

    boolean shouldFeed = false;
    if (shooterState == ShooterState.PASSING) {
      shouldFeed = shooter.isAtGoalSpeed();
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
    // latch: once feeding starts, keep going for at least FEED_LATCH_TIME
    double now = Timer.getFPGATimestamp();
    if (shouldFeed && feedLatchStartTime < 0) {
      feedLatchStartTime = now;
    }
    if (!shouldFeed && feedLatchStartTime > 0 && (now - feedLatchStartTime) < FEED_LATCH_TIME) {
      shouldFeed = true; // hold feed during latch window
    }

    if (shouldFeed) {
      IndexerSubsystem.getInstance().feed();
    }

    Logger.recordOutput("Handoff/ShouldFeed", shouldFeed);
    Logger.recordOutput("Handoff/Intaking", intaking);
    Logger.recordOutput("Handoff/IntakeState", intakeState.name());

    if (!shouldFeed) {
      if (intaking && !currentBeamBroken) {
        // Stage: slowly pull ball to beam break so intake can hold another
        handoffMotor.setControl(velocityRequest.withVelocity(STAGE_RPS));
        IndexerSubsystem.getInstance().feed();
      } else {
        handoffMotor.setControl(neutralRequest);
        IndexerSubsystem.getInstance().stop();
      }
      resetJamState();
      feedLatchStartTime = -1.0;
      return;
    }

    if (unjamming) {
      handoffMotor.setControl(velocityRequest.withVelocity(REVERSE_RPS));
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

    handoffMotor.setControl(velocityRequest.withVelocity(FEED_RPS));
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
