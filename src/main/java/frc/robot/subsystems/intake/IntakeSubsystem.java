package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
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

  private static final int DEPLOY_MOTOR_ID = 31;
  private static final int ROLLER_MOTOR_ID = 30;

  private static final double STOW_POSITION = 0.0;
  private static final double DEPLOY_POSITION = 4.6679983; // tune

  private static final double DEPLOY_P = 0.1;
  private static final double DEPLOY_I = 0.01; // tune
  private static final double DEPLOY_D = 0.0; // tune — dampen oscillation
  static final double kS = 0.00;
  static final double kV = 0.104;
  static final double kA = 0.001;

  private static final double INTAKE_VOLTAGE = 6.0;
  private static final double EJECT_VOLTAGE = -6.0; // tune

  private final TalonFX deployMotor = new TalonFX(DEPLOY_MOTOR_ID);
  private final MotionMagicVoltage motionMagicRequest = new MotionMagicVoltage(0);

  private final SparkMax rollerMotor = new SparkMax(ROLLER_MOTOR_ID, MotorType.kBrushless);

  private IntakeSubsystem() {
    var config = new TalonFXConfiguration();
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    // 70a burst for 1.5s drop to 35a sustained
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = 70.0;
    config.CurrentLimits.SupplyCurrentLowerLimit = 40.0;
    config.CurrentLimits.SupplyCurrentLowerTime = 1.5;
    config.CurrentLimits.StatorCurrentLimit = 120.0;
    config.CurrentLimits.StatorCurrentLimitEnable = true;

    config.Slot0.kS = kS;
    config.Slot0.kP = DEPLOY_P;
    config.Slot0.kI = DEPLOY_I;
    config.Slot0.kD = DEPLOY_D;

    config.MotionMagic.MotionMagicCruiseVelocity = 90; // rps
    config.MotionMagic.MotionMagicAcceleration = 7; // 7 rps^2

    config.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
    config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = DEPLOY_POSITION;
    config.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;
    config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0.0; // 0 rotations

    deployMotor.getConfigurator().apply(config);

    SparkMaxConfig rollerConfig = new SparkMaxConfig();
    rollerConfig.idleMode(IdleMode.kCoast).smartCurrentLimit(35).inverted(false);
    rollerMotor.configure(
        rollerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    deployMotor.setPosition(0.0); // assume starting stowed
  }

  @Override
  public void periodic() {
    switch (state) {
      case STOWED:
        deployMotor.setControl(new PositionDutyCycle(STOW_POSITION).withSlot(0));
        rollerMotor.setVoltage(0.0);
        break;
      case DEPLOYING:
        //        deployMotor.setControl(motionMagicRequest.withPosition(DEPLOY_POSITION));
        deployMotor.setControl(new PositionDutyCycle(DEPLOY_POSITION).withSlot(0));
        rollerMotor.setVoltage(0.0);
        break;
      case INTAKING:
        //        deployMotor.setControl(motionMagicRequest.withPosition(DEPLOY_POSITION));
        deployMotor.setControl(new PositionDutyCycle(DEPLOY_POSITION).withSlot(0));
        rollerMotor.setVoltage(INTAKE_VOLTAGE);
        break;
      case EJECTING:
        //        deployMotor.setControl(motionMagicRequest.withPosition(DEPLOY_POSITION));
        deployMotor.setControl(new PositionDutyCycle(DEPLOY_POSITION).withSlot(0));
        rollerMotor.setVoltage(EJECT_VOLTAGE);
        break;
      case IDLE_DEPLOYED:
        //        deployMotor.setControl(motionMagicRequest.withPosition(DEPLOY_POSITION));
        deployMotor.setControl(new PositionDutyCycle(DEPLOY_POSITION).withSlot(0));
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
    return deployMotor.getPosition().getValueAsDouble();
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
