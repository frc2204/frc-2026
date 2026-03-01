package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
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
import edu.wpi.first.wpilibj.Timer;
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
  private static final double GEAR_RATIO = 19.8;

  private static final double STOW_POSITION = 0.0;
  private static final double DEPLOY_POSITION = 0.25; // ~90 degrees in mechanism rotations

  // Slot 0 — deploy (down)
  private static final double DEPLOY_P = 0.5;
  private static final double DEPLOY_I = 0.0; // tune
  private static final double DEPLOY_D = 0.0; // tune
  // Slot 1 — stow (up)
  private static final double STOW_P = 1.5; // tune
  private static final double STOW_I = 0.0; // tune
  private static final double STOW_D = 0.0; // tune

  static final double kS = 0.00;
  static final double kV = 0.104;
  static final double kA = 0.001;

  private static final double INTAKE_RPM = 325.0; // tune
  private static final double EJECT_RPM = -325.0; // tune
  private static final double KICK_VOLTAGE = 12.0; // full voltage kick
  private static final double KICK_DURATION = 1.0; // seconds before switching to PID

  private double intakeStartTime = -1.0;

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
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = 30.0;
    config.CurrentLimits.SupplyCurrentLowerLimit = 20.0;
    config.CurrentLimits.SupplyCurrentLowerTime = 1.0;
    config.CurrentLimits.StatorCurrentLimit = 40.0; // low to protect 3D print
    config.CurrentLimits.StatorCurrentLimitEnable = true;

    config.Slot0.kS = kS;
    config.Slot0.kP = DEPLOY_P;
    config.Slot0.kI = DEPLOY_I;
    config.Slot0.kD = DEPLOY_D;

    config.Slot1.kS = kS;
    config.Slot1.kP = STOW_P;
    config.Slot1.kI = STOW_I;
    config.Slot1.kD = STOW_D;

    config.Feedback.SensorToMechanismRatio = GEAR_RATIO;

    config.MotionMagic.MotionMagicCruiseVelocity = 0.5; // mechanism rps (~180 deg/s)
    config.MotionMagic.MotionMagicAcceleration = 1.0; // mechanism rps^2

    config.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
    config.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;

    deployMotor.getConfigurator().apply(config);
    com.ctre.phoenix6.hardware.ParentDevice.optimizeBusUtilizationForAll(deployMotor);

    SparkMaxConfig rollerConfig = new SparkMaxConfig();
    rollerConfig.idleMode(IdleMode.kCoast).smartCurrentLimit(40).inverted(false);
    rollerConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(ROLLER_P, ROLLER_I, ROLLER_D)
        .feedForward
        .kV(ROLLER_KV);
    rollerConfig
        .signals
        .primaryEncoderPositionPeriodMs(500)
        .primaryEncoderVelocityPeriodMs(20) // needed for PID
        .analogVoltagePeriodMs(500)
        .analogPositionPeriodMs(500)
        .analogVelocityPeriodMs(500)
        .appliedOutputPeriodMs(100)
        .busVoltagePeriodMs(500)
        .outputCurrentPeriodMs(500);
    rollerMotor.configure(
        rollerConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    deployMotor.setPosition(0.0); // assume starting stowed
  }

  @Override
  public void periodic() {
    switch (state) {
      case STOWED:
        // deployMotor.setControl(motionMagicRequest.withPosition(STOW_POSITION).withSlot(1));
        rollerMotor.setVoltage(0.0);
        break;
      case DEPLOYING:
        // deployMotor.setControl(motionMagicRequest.withPosition(DEPLOY_POSITION).withSlot(0));
        rollerMotor.setVoltage(0.0);
        break;
      case INTAKING:
        // deployMotor.setControl(motionMagicRequest.withPosition(DEPLOY_POSITION).withSlot(0));
        if (Timer.getFPGATimestamp() - intakeStartTime < KICK_DURATION) {
          rollerMotor.setVoltage(KICK_VOLTAGE);
        } else {
          rollerPID.setSetpoint(INTAKE_RPM, ControlType.kVelocity);
        }
        break;
      case EJECTING:
        // deployMotor.setControl(motionMagicRequest.withPosition(DEPLOY_POSITION).withSlot(0));
        if (Timer.getFPGATimestamp() - intakeStartTime < KICK_DURATION) {
          rollerMotor.setVoltage(-KICK_VOLTAGE);
        } else {
          rollerPID.setSetpoint(EJECT_RPM, ControlType.kVelocity);
        }
        break;
      case IDLE_DEPLOYED:
        // deployMotor.setControl(motionMagicRequest.withPosition(DEPLOY_POSITION).withSlot(0));
        rollerMotor.setVoltage(0.0);
        break;
    }
  }

  public void setState(IntakeState newState) {
    if ((newState == IntakeState.INTAKING && state != IntakeState.INTAKING)
        || (newState == IntakeState.EJECTING && state != IntakeState.EJECTING)) {
      intakeStartTime = Timer.getFPGATimestamp();
    }
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
