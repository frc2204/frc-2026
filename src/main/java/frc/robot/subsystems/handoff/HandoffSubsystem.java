package frc.robot.subsystems.handoff;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.intake.IndexerSubsystem;
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

  private static final double FEED_VOLTAGE_PEAK = -12.0; // first second burst
  private static final double FEED_VOLTAGE_STEADY = -12.0; // after 1s
  private static final double PEAK_DURATION = 1.0; // seconds
  private static final double REVERSE_VOLTAGE = 4.0; // tune

  // if beam break is blocked for this long reverse
  private static final double JAM_TIME_SECONDS = 0.5; // tune
  private static final double UNJAM_TIME_SECONDS = 0.3; // tune
  private static final double FEED_LATCH_TIME =
      0.5; // seconds — min time to keep feeding. 0 = no latch

  private final TalonFX handoffMotor = new TalonFX(HANDOFF_MOTOR_ID);
  private final VoltageOut voltageRequest = new VoltageOut(0.0).withEnableFOC(true);
  private final DigitalInput beamBreak = new DigitalInput(BEAM_BREAK_DIO);
  private boolean forceReverse = false;

  private double feedStartTime = -1.0;
  private double feedLatchStartTime = -1.0;
  private double blockedStartTime = -1.0;
  private double unjamStartTime = -1.0;
  private boolean unjamming = false;

  private boolean lastBeamBroken = false;
  private int ballsScored = 0;

  private HandoffSubsystem() {
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.StatorCurrentLimit = 60;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = 40; // peak spike
    config.CurrentLimits.SupplyCurrentLowerLimit = 25; // steady state after spike
    config.CurrentLimits.SupplyCurrentLowerTime = 3.0; // seconds at peak before dropping
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
      handoffMotor.setControl(voltageRequest.withOutput(REVERSE_VOLTAGE));
      resetJamState();
      return;
    }

    ShooterSubsystem shooter = ShooterSubsystem.getInstance();
    ShooterState shooterState = shooter.getState();

    boolean shouldFeed = false;
    if (shooterState == ShooterState.PASSING) {
      shouldFeed = true; // full voltage, feed
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

    if (!shouldFeed) {
      handoffMotor.setControl(voltageRequest.withOutput(0.0));
      IndexerSubsystem.getInstance().stop();
      resetJamState();
      feedStartTime = -1.0;
      feedLatchStartTime = -1.0;
      return;
    }

    if (feedStartTime < 0) {
      feedStartTime = Timer.getFPGATimestamp();
    }

    //    double now = Timer.getFPGATimestamp();

    if (unjamming) {
      handoffMotor.setControl(voltageRequest.withOutput(REVERSE_VOLTAGE));
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

    double feedVoltage =
        (now - feedStartTime < PEAK_DURATION) ? FEED_VOLTAGE_PEAK : FEED_VOLTAGE_STEADY;
    handoffMotor.setControl(voltageRequest.withOutput(feedVoltage));
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
