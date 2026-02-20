package frc.robot.subsystems.intake;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {

  private static final IntakeSubsystem INSTANCE = new IntakeSubsystem();

  public static IntakeSubsystem getInstance() {
    return INSTANCE;
  }

  public enum IntakeState {
    STOWED,
    DEPLOYING,
    INTAKING,
    EJECTING,
    IDLE_DEPLOYED
  }

  private IntakeState state = IntakeState.STOWED;

  private static final int DEPLOY_MOTOR_ID = 20;
  private static final int ROLLER_MOTOR_ID = 21;

  private static final double STOW_POSITION = 0.0;
  private static final double DEPLOY_POSITION = 5.0; // tune

  private static final double DEPLOY_P = 0.1;
  private static final double DEPLOY_I = 0.0; // tune
  private static final double DEPLOY_D = 0.0;

  private static final double INTAKE_VOLTAGE = 8.0;
  private static final double EJECT_VOLTAGE = -6.0; // tune

  private final SparkMax deployMotor = new SparkMax(DEPLOY_MOTOR_ID, MotorType.kBrushless);
  private final SparkMax rollerMotor = new SparkMax(ROLLER_MOTOR_ID, MotorType.kBrushless);
  private final SparkClosedLoopController deployController;
  private final RelativeEncoder deployEncoder;

  private IntakeSubsystem() {
    SparkMaxConfig deployConfig = new SparkMaxConfig();
    deployConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(30)
        .inverted(false)
        .closedLoop
        .pid(DEPLOY_P, DEPLOY_I, DEPLOY_D)
        .outputRange(-1.0, 1.0);
    deployMotor.configure(
        deployConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    SparkMaxConfig rollerConfig = new SparkMaxConfig();
    rollerConfig.idleMode(IdleMode.kCoast).smartCurrentLimit(30).inverted(false);
    rollerMotor.configure(
        rollerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    deployController = deployMotor.getClosedLoopController();
    deployEncoder = deployMotor.getEncoder();
    deployEncoder.setPosition(0.0); // assume starting stowed
  }

  @Override
  public void periodic() {
    switch (state) {
      case STOWED:
        deployController.setReference(STOW_POSITION, ControlType.kPosition);
        rollerMotor.setVoltage(0.0);
        break;
      case DEPLOYING:
        deployController.setReference(DEPLOY_POSITION, ControlType.kPosition);
        rollerMotor.setVoltage(0.0);
        break;
      case INTAKING:
        deployController.setReference(DEPLOY_POSITION, ControlType.kPosition);
        rollerMotor.setVoltage(INTAKE_VOLTAGE);
        break;
      case EJECTING:
        deployController.setReference(DEPLOY_POSITION, ControlType.kPosition);
        rollerMotor.setVoltage(EJECT_VOLTAGE);
        break;
      case IDLE_DEPLOYED:
        deployController.setReference(DEPLOY_POSITION, ControlType.kPosition);
        rollerMotor.setVoltage(0.0);
        break;
    }
  }

  public void setState(IntakeState newState) {
    state = newState;
  }

  public IntakeState getState() {
    return state;
  }

  public double getDeployPosition() {
    return deployEncoder.getPosition();
  }

  public void stow() {
    setState(IntakeState.STOWED);
  }

  public void intake() {
    setState(IntakeState.INTAKING);
  }

  public void eject() {
    setState(IntakeState.EJECTING);
  }
}
