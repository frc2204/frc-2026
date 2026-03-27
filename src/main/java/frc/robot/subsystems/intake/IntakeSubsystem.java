package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
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
    EJECTING,
    IDLE_DEPLOYED,
    MANUAL
  }

  private IntakeState state = IntakeState.STOWED;

  private static final int DEPLOY_RIGHT_MOTOR_ID = 31;
  private static final int DEPLOY_LEFT_MOTOR_ID = 32;
  private static final int ROLLER_MOTOR_ID = 30;
  private static final double GEAR_RATIO = 1;

  private static final double STOW_POSITION = 0.5;
  private static final double DEPLOY_POSITION = -23.40; // ~90 degrees in mechanism rotations

  // Slot 0 — deploy (down)
  private static final double DEPLOY_KS = 0.37; // tune
  private static final double DEPLOY_P = 1; // tune
  private static final double DEPLOY_D = 0.0; // tune
  // Slot 1 — stow (up)
  private static final double STOW_KS = 0.37; // tune
  private static final double STOW_P = 1.5; // tune
  private static final double STOW_D = 0.0; // tune

  private static final double INTAKE_RPS = 1000.0 / 60.0; // tune
  private static final double EJECT_RPS = -1000.0 / 60.0; // tune

  // roller VelocityTorqueCurrentFOC PID (units: amps)
  private static final double ROLLER_KS = 3.0; // amps — overcome friction, tune
  private static final double ROLLER_KP = 3.0; // amps per RPS error, tune
  private static final double ROLLER_KD = 0.0; // tune

  private final TalonFX deployRightMotor = new TalonFX(DEPLOY_RIGHT_MOTOR_ID);
  private final TalonFX deployLeftMotor = new TalonFX(DEPLOY_LEFT_MOTOR_ID);
  private final MotionMagicVoltage motionMagicRequest =
      new MotionMagicVoltage(0).withEnableFOC(true);

  private final TalonFX rollerMotor = new TalonFX(ROLLER_MOTOR_ID);
  private final VelocityTorqueCurrentFOC rollerVelocityRequest = new VelocityTorqueCurrentFOC(0);
  private final MotionMagicVoltage motionMagicRequestLeft =
      new MotionMagicVoltage(0).withEnableFOC(true);
  private final NeutralOut neutralRequest = new NeutralOut();
  private final VoltageOut voltageRequest = new VoltageOut(0).withEnableFOC(true);
  private double manualVoltage = 0.0;

  private static final double MAX_POSITION_DIVERGENCE = 3.0; // rotations, tune
  private boolean deployStalled = false;

  private IntakeSubsystem() {
    var config = new TalonFXConfiguration();
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = 30.0;
    config.CurrentLimits.SupplyCurrentLowerLimit = 20.0;
    config.CurrentLimits.SupplyCurrentLowerTime = 1.0;
    //    config.CurrentLimits.StatorCurrentLimit = 40.0; // low to protect 3D print
    //    config.CurrentLimits.StatorCurrentLimitEnable = true;

    config.Slot0.kS = DEPLOY_KS;
    config.Slot0.kP = DEPLOY_P;
    config.Slot0.kD = DEPLOY_D;

    config.Slot1.kS = STOW_KS;
    config.Slot1.kP = STOW_P;
    config.Slot1.kD = STOW_D;

    //    config.Feedback.SensorToMechanismRatio = GEAR_RATIO;

    config.MotionMagic.MotionMagicCruiseVelocity = 40; // mechanism rps (~180 deg/s)
    config.MotionMagic.MotionMagicAcceleration = 40; // mechanism rps^2

    config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

    deployRightMotor.getConfigurator().apply(config);

    // Second deploy motor — independent, opposed direction
    var deployConfig2 = new TalonFXConfiguration();
    deployConfig2.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    deployConfig2.MotorOutput.Inverted = InvertedValue.Clockwise_Positive; // opposed
    deployConfig2.CurrentLimits.SupplyCurrentLimitEnable = true;
    deployConfig2.CurrentLimits.SupplyCurrentLimit = 30.0;
    deployConfig2.CurrentLimits.SupplyCurrentLowerLimit = 20.0;
    deployConfig2.CurrentLimits.SupplyCurrentLowerTime = 1.0;
    deployConfig2.Slot0.kS = DEPLOY_KS;
    deployConfig2.Slot0.kP = DEPLOY_P;
    deployConfig2.Slot0.kD = DEPLOY_D;
    deployConfig2.Slot1.kS = STOW_KS;
    deployConfig2.Slot1.kP = STOW_P;
    deployConfig2.Slot1.kD = STOW_D;
    deployConfig2.MotionMagic.MotionMagicCruiseVelocity = 40;
    deployConfig2.MotionMagic.MotionMagicAcceleration = 40;
    deployConfig2.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
    deployConfig2.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;
    deployLeftMotor.getConfigurator().apply(deployConfig2);
    deployLeftMotor.setPosition(0.0);

    // Roller motor — Kraken X44
    var rollerConfig = new TalonFXConfiguration();
    rollerConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    rollerConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    //    rollerConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    //    rollerConfig.CurrentLimits.StatorCurrentLimit = 40; // reduced for Kraken X44
    rollerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    rollerConfig.CurrentLimits.SupplyCurrentLimit = 60; // reduced for Kraken X44
    rollerConfig.CurrentLimits.SupplyCurrentLowerLimit = 30.0;
    rollerConfig.CurrentLimits.SupplyCurrentLowerTime = 1.0;
    rollerConfig.Slot0.kS = ROLLER_KS;
    rollerConfig.Slot0.kP = ROLLER_KP;
    rollerConfig.Slot0.kD = ROLLER_KD;
    rollerMotor.getConfigurator().apply(rollerConfig);

    deployRightMotor.setPosition(0.0); // assume starting stowed
  }

  @Override
  public void periodic() {
    // Check if deploy motors have diverged (one side stuck)
    double rightPos = deployRightMotor.getPosition().getValueAsDouble();
    double leftPos = deployLeftMotor.getPosition().getValueAsDouble();
    double divergence = Math.abs(rightPos - leftPos);
    if (divergence > MAX_POSITION_DIVERGENCE && state != IntakeState.MANUAL) {
      deployStalled = true;
    }
    // Clear stall if positions converge again (e.g. after manual intervention)
    if (divergence < MAX_POSITION_DIVERGENCE * 0.5) {
      deployStalled = false;
    }

    if (deployStalled) {
      deployRightMotor.setControl(neutralRequest);
      deployLeftMotor.setControl(neutralRequest);
      rollerMotor.setControl(neutralRequest);
      Logger.recordOutput("Intake/DeployStalled", true);
      Logger.recordOutput("Intake/Divergence", divergence);
      Logger.recordOutput("Intake/RightPos", rightPos);
      Logger.recordOutput("Intake/LeftPos", leftPos);
      return;
    }

    switch (state) {
      case STOWED:
        deployRightMotor.setControl(motionMagicRequest.withPosition(STOW_POSITION).withSlot(1));
        deployLeftMotor.setControl(motionMagicRequestLeft.withPosition(STOW_POSITION).withSlot(1));
        rollerMotor.setControl(neutralRequest);
        break;
      case DEPLOYING:
        deployRightMotor.setControl(motionMagicRequest.withPosition(DEPLOY_POSITION).withSlot(0));
        deployLeftMotor.setControl(
            motionMagicRequestLeft.withPosition(DEPLOY_POSITION).withSlot(0));
        rollerMotor.setControl(neutralRequest);
        break;
      case INTAKING:
        deployRightMotor.setControl(motionMagicRequest.withPosition(DEPLOY_POSITION).withSlot(0));
        deployLeftMotor.setControl(
            motionMagicRequestLeft.withPosition(DEPLOY_POSITION).withSlot(0));
        rollerMotor.setControl(rollerVelocityRequest.withVelocity(INTAKE_RPS));
        break;
      case EJECTING:
        deployRightMotor.setControl(motionMagicRequest.withPosition(DEPLOY_POSITION).withSlot(0));
        deployLeftMotor.setControl(
            motionMagicRequestLeft.withPosition(DEPLOY_POSITION).withSlot(0));
        rollerMotor.setControl(rollerVelocityRequest.withVelocity(EJECT_RPS));
        break;
      case IDLE_DEPLOYED:
        deployRightMotor.setControl(motionMagicRequest.withPosition(DEPLOY_POSITION).withSlot(0));
        deployLeftMotor.setControl(
            motionMagicRequestLeft.withPosition(DEPLOY_POSITION).withSlot(0));
        rollerMotor.setControl(neutralRequest);
        break;
      case MANUAL:
        deployRightMotor.setControl(voltageRequest.withOutput(manualVoltage));
        deployLeftMotor.setControl(voltageRequest.withOutput(manualVoltage));
        rollerMotor.setControl(neutralRequest);
        break;
    }

    Logger.recordOutput("Intake/DeployStalled", false);
    Logger.recordOutput("Intake/Divergence", divergence);
    Logger.recordOutput("Intake/RightPos", rightPos);
    Logger.recordOutput("Intake/LeftPos", leftPos);
    Logger.recordOutput(
        "Intake/Deploy2SupplyCurrent", deployLeftMotor.getSupplyCurrent().getValueAsDouble());
    Logger.recordOutput(
        "Intake/Deploy2StatorCurrent", deployLeftMotor.getStatorCurrent().getValueAsDouble());
  }

  public void setState(IntakeState newState) {
    state = newState;
  }

  public IntakeState getState() {
    return state;
  }

  public double getDeployPosition() {
    return deployRightMotor.getPosition().getValueAsDouble();
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

  public void setManualVoltage(double voltage) {
    manualVoltage = voltage;
  }
}
