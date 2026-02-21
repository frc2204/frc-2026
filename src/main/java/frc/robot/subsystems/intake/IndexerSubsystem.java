package frc.robot.subsystems.intake;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
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

  // CAN ID — change once wired
  private static final int INDEXER_MOTOR_ID = 23;

  // Voltages
  private static final double FEED_VOLTAGE = 6.0; // tune
  private static final double REVERSE_VOLTAGE = -4.0; // tune

  private final SparkMax indexerMotor = new SparkMax(INDEXER_MOTOR_ID, MotorType.kBrushless);

  private IndexerSubsystem() {
    SparkMaxConfig config = new SparkMaxConfig();
    config.idleMode(IdleMode.kBrake).smartCurrentLimit(30).inverted(false);
    indexerMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
    switch (state) {
      case FEEDING:
        indexerMotor.setVoltage(FEED_VOLTAGE);
        break;
      case REVERSING:
        indexerMotor.setVoltage(REVERSE_VOLTAGE);
        break;
      case STOPPED:
        indexerMotor.setVoltage(0.0);
        break;
    }
  }

  public void setState(IndexerState newState) {
    state = newState;
  }

  public IndexerState getState() {
    return state;
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
