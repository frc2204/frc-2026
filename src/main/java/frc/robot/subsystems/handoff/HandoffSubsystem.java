package frc.robot.subsystems.handoff;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.shooting.ShooterSubsystem;
import frc.robot.subsystems.shooting.ShooterSubsystem.ShooterState;

public class HandoffSubsystem extends SubsystemBase {

  private static final HandoffSubsystem INSTANCE = new HandoffSubsystem();

  public static HandoffSubsystem getInstance() {
    return INSTANCE;
  }

  private static final int HANDOFF_MOTOR_ID = 22;

  private static final double FEED_VOLTAGE = 8.0; // tune
  private static final double REVERSE_VOLTAGE = -4.0; // tune

  private final SparkMax handoffMotor = new SparkMax(HANDOFF_MOTOR_ID, MotorType.kBrushless);
  private boolean forceReverse = false;

  private HandoffSubsystem() {
    SparkMaxConfig config = new SparkMaxConfig();
    config.idleMode(IdleMode.kBrake).smartCurrentLimit(30).inverted(false);
    handoffMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
    if (forceReverse) {
      handoffMotor.setVoltage(REVERSE_VOLTAGE);
      return;
    }

    ShooterSubsystem shooter = ShooterSubsystem.getInstance();
    ShooterState shooterState = shooter.getState();

    boolean shouldFeed = false;
    if (shooterState == ShooterState.RAPID_FIRE) {
      shouldFeed = shooter.isAtGoalSpeed();
    } else if (shooterState == ShooterState.RAPID_FIRE_ACCURATE) {
      shouldFeed = shooter.isAtGoalSpeedAccurate();
    }

    handoffMotor.setVoltage(shouldFeed ? FEED_VOLTAGE : 0.0);
  }

  public void setReverse(boolean reverse) {
    forceReverse = reverse;
  }
}
