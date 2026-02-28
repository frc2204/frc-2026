package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.FeedbackSensor;
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

  private static final int DEPLOY_MOTOR_ID = 31;
  private static final int ROLLER_MOTOR_ID = 30;

  private static final double STOW_POSITION = 0.0;
  private static final double DEPLOY_POSITION = 4.6679983; // tune

  // Slot 0 — deploy (down)
  private static final double DEPLOY_P = 0.1;
  private static final double DEPLOY_I = 0.01; // tune
  private static final double DEPLOY_D = 0.0; // tune
  // Slot 1 — stow (up)
  private static final double STOW_P = 0.6; // tune
  private static final double STOW_I = 0.01; // tune
  private static final double STOW_D = 0.0; // tune

  static final double kS = 0.00;
  static final double kV = 0.104;
  static final double kA = 0.001;

  private static final double INTAKE_RPM = 3000.0; // tune
  private static final double EJECT_RPM = -3000.0; // tune

  // roller velocity PID
  private static final double ROLLER_P = 0.0002; // tune
  private static final double ROLLER_I = 0.0;
  private static final double ROLLER_D = 0.0;
  private static final double ROLLER_KV = 12.0 / 5676.0; // volts per RPM, NEO free speed ~5676 RPM

  private final TalonFX deployMotor = new TalonFX(DEPLOY_MOTOR_ID);
  private final MotionMagicVoltage motionMagicRequest = new MotionMagicVoltage(0);

  private final SparkMax rollerMotor = new SparkMax(ROLLER_MOTOR_ID, MotorType.kBrushless);
  private final SparkClosedLoopController rollerPID = rollerMotor.getClosedLoopController();

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

    config.Slot1.kS = kS;
    config.Slot1.kP = STOW_P;
    config.Slot1.kI = STOW_I;
    config.Slot1.kD = STOW_D;

    config.MotionMagic.MotionMagicCruiseVelocity = 90; // rps
    config.MotionMagic.MotionMagicAcceleration = 7; // 7 rps^2

    config.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
    config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = DEPLOY_POSITION;
    config.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;
    config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0.0; // 0 rotations

    deployMotor.getConfigurator().apply(config);

    SparkMaxConfig rollerConfig = new SparkMaxConfig();
    rollerConfig.idleMode(IdleMode.kCoast).smartCurrentLimit(35).inverted(false);
    rollerConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(ROLLER_P, ROLLER_I, ROLLER_D)
        .feedForward
        .kV(ROLLER_KV);
    rollerMotor.configure(
        rollerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    deployMotor.setPosition(0.0); // assume starting stowed
  }

  @Override
  public void periodic() {
    switch (state) {
      case STOWED:
        deployMotor.setVoltage(0.0);
        rollerMotor.setVoltage(0.0);
        break;
      case DEPLOYING:
        deployMotor.setControl(new PositionDutyCycle(DEPLOY_POSITION).withSlot(0));
        rollerMotor.setVoltage(0.0);
        break;
      case INTAKING:
        deployMotor.setControl(new PositionDutyCycle(DEPLOY_POSITION).withSlot(0));
        rollerPID.setSetpoint(INTAKE_RPM, ControlType.kVelocity);
        break;
      case EJECTING:
        deployMotor.setControl(new PositionDutyCycle(DEPLOY_POSITION).withSlot(0));
        rollerPID.setSetpoint(EJECT_RPM, ControlType.kVelocity);
        break;
      case IDLE_DEPLOYED:
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
