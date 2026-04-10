package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class IntakeSubsystem extends SubsystemBase {

  private static final IntakeSubsystem INSTANCE = new IntakeSubsystem();

  public static IntakeSubsystem getInstance() {
    return INSTANCE;
  }

  public enum IntakeState {
    STOWED,
    DEPLOYING,
    INTAKING,
    DEPOT_INTAKING,
    EJECTING,
    IDLE_DEPLOYED,
    MANUAL
  }

  private IntakeState state = IntakeState.STOWED;

  private static final int ROLLER_MOTOR_ID = 31;
  private static final int DEPLOY_MOTOR_ID = 32;

  private static final double GEAR_RATIO = 15.0; // 1:15 motor:mechanism

  private static final double STOW_POSITION = 0.023;
  private static final double DEPLOY_POSITION = 0.228516; // ~90 degrees in mechanism rotations
  private static final double DEPOT_DEPLOY_POSITION = 0.166553; // TUNE — depot intake angle

  // Slot 0 — deploy (down)
  private static final double DEPLOY_KS = 0.25; // tune
  private static final double DEPLOY_KP = 50.5; // tune
  private static final double DEPLOY_KD = 0.1; // tune — dampens slam
  private static final double DEPLOY_KG =
      0.5; // tune — gravity compensation (volts to hold horizontal)

  // Slot 1 — stow (up)
  private static final double STOW_KS = 0.37; // tune
  private static final double STOW_KP = 18.5; // tune
  private static final double STOW_KD = 0.0; // tune
  private static final double STOW_KG = 0.3; // tune — gravity compensation

  private static final double INTAKE_RPS = 2500.0 / 60.0; // tune
  private static final double EJECT_RPS = -2500.0 / 60.0; // tune

  // roller VelocityTorqueCurrentFOC PID (units: amps)
  private static final double ROLLER_KS = 3.0; // amps — overcome friction, tune
  private static final double ROLLER_KP = 3.0; // amps per RPS error, tune
  private static final double ROLLER_KD = 0.0; // tune

  private final TalonFX deployMotor = new TalonFX(DEPLOY_MOTOR_ID);
  private final MotionMagicVoltage motionMagicRequest =
      new MotionMagicVoltage(0).withEnableFOC(true);

  private final TalonFX rollerMotor = new TalonFX(ROLLER_MOTOR_ID);
  private final VelocityTorqueCurrentFOC rollerVelocityRequest = new VelocityTorqueCurrentFOC(0);
  private final NeutralOut neutralRequest = new NeutralOut();
  private final VoltageOut voltageRequest = new VoltageOut(0).withEnableFOC(true);
  private double manualVoltage = 0.0;

  private IntakeSubsystem() {
    // Deploy motor config
    var config = new TalonFXConfiguration();
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = 60.0;
    config.CurrentLimits.SupplyCurrentLowerLimit = 30.0;
    config.CurrentLimits.SupplyCurrentLowerTime = 1.0;

    config.Slot0.kS = DEPLOY_KS;
    config.Slot0.kP = DEPLOY_KP;
    config.Slot0.kD = DEPLOY_KD;
    config.Slot0.kG = DEPLOY_KG;
    config.Slot0.GravityType = GravityTypeValue.Arm_Cosine;

    config.Slot1.kS = STOW_KS;
    config.Slot1.kP = STOW_KP;
    config.Slot1.kD = STOW_KD;
    config.Slot1.kG = STOW_KG;
    config.Slot1.GravityType = GravityTypeValue.Arm_Cosine;

    config.Feedback.SensorToMechanismRatio = GEAR_RATIO;

    config.MotionMagic.MotionMagicCruiseVelocity = 0.5; // mechanism rps — slow deploy over ~0.5s
    config.MotionMagic.MotionMagicAcceleration = 2; // mechanism rps^2

    config.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
    config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 0.0;
    config.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;
    config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = DEPLOY_POSITION;

    deployMotor.getConfigurator().apply(config);
    deployMotor.setPosition(0.0); // assume starting stowed

    // Roller motor config
    var rollerConfig = new TalonFXConfiguration();
    rollerConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    rollerConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    rollerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    rollerConfig.CurrentLimits.SupplyCurrentLimit = 60;
    rollerConfig.CurrentLimits.SupplyCurrentLowerLimit = 30.0;
    rollerConfig.CurrentLimits.SupplyCurrentLowerTime = 1.0;
    rollerConfig.Slot0.kS = ROLLER_KS;
    rollerConfig.Slot0.kP = ROLLER_KP;
    rollerConfig.Slot0.kD = ROLLER_KD;
    rollerMotor.getConfigurator().apply(rollerConfig);
  }

  @Override
  public void periodic() {
    double deployPos = deployMotor.getPosition().getValueAsDouble();

    switch (state) {
      case STOWED:
        deployMotor.setControl(motionMagicRequest.withPosition(STOW_POSITION).withSlot(1));
        rollerMotor.setControl(neutralRequest);
        break;
      case DEPLOYING:
        deployMotor.setControl(motionMagicRequest.withPosition(DEPLOY_POSITION).withSlot(0));
        rollerMotor.setControl(neutralRequest);
        break;
      case INTAKING:
        deployMotor.setControl(motionMagicRequest.withPosition(DEPLOY_POSITION).withSlot(0));
        rollerMotor.setControl(rollerVelocityRequest.withVelocity(INTAKE_RPS));
        break;
      case DEPOT_INTAKING:
        deployMotor.setControl(motionMagicRequest.withPosition(DEPOT_DEPLOY_POSITION).withSlot(0));
        rollerMotor.setControl(rollerVelocityRequest.withVelocity(INTAKE_RPS));
        break;
      case EJECTING:
        deployMotor.setControl(motionMagicRequest.withPosition(DEPLOY_POSITION).withSlot(0));
        rollerMotor.setControl(rollerVelocityRequest.withVelocity(EJECT_RPS));
        break;
      case IDLE_DEPLOYED:
        deployMotor.setControl(motionMagicRequest.withPosition(DEPLOY_POSITION).withSlot(0));
        rollerMotor.setControl(neutralRequest);
        break;
      case MANUAL:
        deployMotor.setControl(voltageRequest.withOutput(manualVoltage));
        rollerMotor.setControl(rollerVelocityRequest.withVelocity(INTAKE_RPS));
        break;
    }

    Logger.recordOutput("Intake/DeployPosition", deployPos);
    Logger.recordOutput("Intake/State", state.name());
    Logger.recordOutput(
        "Intake/DeploySupplyCurrent", deployMotor.getSupplyCurrent().getValueAsDouble());
    Logger.recordOutput(
        "Intake/DeployStatorCurrent", deployMotor.getStatorCurrent().getValueAsDouble());
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

  public void depotIntake() {
    setState(IntakeState.DEPOT_INTAKING);
  }

  public void idleDeploy() {
    setState(IntakeState.IDLE_DEPLOYED);
  }

  public void eject() {
    setState(IntakeState.EJECTING);
  }

  public void setManualVoltage(double voltage) {
    manualVoltage = voltage;
  }
}
