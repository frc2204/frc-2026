package frc.robot.subsystems.intake;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IndexerSubsystem extends SubsystemBase {

  private static final IndexerSubsystem INSTANCE = new IndexerSubsystem();

  public static IndexerSubsystem getInstance() {
    return INSTANCE;
  }

  public enum IndexerState {
    STOPPED,
    FEEDING,
    REVERSING
  }

  private IndexerState state = IndexerState.STOPPED;

  private static final int INDEXER_MOTOR_ID = 40; // tune

  private static final double FEED_VOLTAGE_PEAK = -12.0; // first second burst
  private static final double FEED_VOLTAGE_STEADY = -6.0; // after 1s
  private static final double PEAK_DURATION = 1.0; // seconds
  private static final double RAMP_DURATION = 1.0; // seconds to ramp up
  private static final double REVERSE_VOLTAGE = 12.0; // tune

  private final SparkMax indexerMotor = new SparkMax(INDEXER_MOTOR_ID, MotorType.kBrushless);
  private double feedStartTime = -1.0;

  private IndexerSubsystem() {
    SparkMaxConfig config = new SparkMaxConfig();
    config.idleMode(IdleMode.kCoast).smartCurrentLimit(40).inverted(false);
    config
        .signals
        .primaryEncoderPositionPeriodMs(500)
        .primaryEncoderVelocityPeriodMs(500)
        .analogVoltagePeriodMs(500)
        .analogPositionPeriodMs(500)
        .analogVelocityPeriodMs(500)
        .appliedOutputPeriodMs(100)
        .busVoltagePeriodMs(500)
        .outputCurrentPeriodMs(500);
    indexerMotor.configure(
        config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  @Override
  public void periodic() {
    switch (state) {
      case FEEDING:
        if (feedStartTime < 0) {
          feedStartTime = Timer.getFPGATimestamp();
        }
        double elapsed = Timer.getFPGATimestamp() - feedStartTime;
        double targetVoltage = (elapsed < PEAK_DURATION) ? FEED_VOLTAGE_PEAK : FEED_VOLTAGE_STEADY;
        double rampScale = Math.min(elapsed / RAMP_DURATION, 1.0);
        indexerMotor.setVoltage(targetVoltage * rampScale);
        break;
      case REVERSING:
        indexerMotor.setVoltage(REVERSE_VOLTAGE);
        feedStartTime = -1.0;
        break;
      case STOPPED:
        indexerMotor.setVoltage(0.0);
        feedStartTime = -1.0;
        break;
    }
  }

  public void setState(IndexerState newState) {
    state = newState;
  }

  public IndexerState getState() {
    return state;
  }

  /** Briefly pulse the motor to wake up the SparkMax controller. */
  public void wakeUp() {
    indexerMotor.setVoltage(0.01);
    indexerMotor.setVoltage(0.0);
  }

  public void feed() {
    setState(IndexerState.FEEDING);
  }

  public void stop() {
    setState(IndexerState.STOPPED);
  }

  public void reverse() {
    setState(IndexerState.REVERSING);
  }
}
