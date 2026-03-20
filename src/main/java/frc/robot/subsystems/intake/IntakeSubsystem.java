package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
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
    IDLE_DEPLOYED
  }

  private IntakeState state = IntakeState.STOWED;

  private static final int DEPLOY_RIGHT_MOTOR_ID = 31;
  private static final int DEPLOY_LEFT_MOTOR_ID = 32;
  private static final int ROLLER_MOTOR_ID = 30;
  private static final double GEAR_RATIO = 1;

  private static final double STOW_POSITION = 0.0;
  private static final double DEPLOY_POSITION = -23.47; // ~90 degrees in mechanism rotations

  // Slot 0 — deploy (down)
  private static final double DEPLOY_KS = 0.0; // tune
  private static final double DEPLOY_P = 0.5; // tune
  private static final double DEPLOY_D = 0.0; // tune
  // Slot 1 — stow (up)
  private static final double STOW_KS = 0.0; // tune
  private static final double STOW_P = 1.5; // tune
  private static final double STOW_D = 0.0; // tune

  private static final double INTAKE_RPS = 325.0 / 60.0; // tune
  private static final double EJECT_RPS = -325.0 / 60.0; // tune

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
  private final NeutralOut neutralRequest = new NeutralOut();

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

    config.Slot0.kS = DEPLOY_KS;
    config.Slot0.kP = DEPLOY_P;
    config.Slot0.kD = DEPLOY_D;

    config.Slot1.kS = STOW_KS;
    config.Slot1.kP = STOW_P;
    config.Slot1.kD = STOW_D;

    //    config.Feedback.SensorToMechanismRatio = GEAR_RATIO;

    config.MotionMagic.MotionMagicCruiseVelocity = 0.5; // mechanism rps (~180 deg/s)
    config.MotionMagic.MotionMagicAcceleration = 1.0; // mechanism rps^2

    config.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
    config.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;

    deployRightMotor.getConfigurator().apply(config);

    // Second deploy motor — opposed follower of deployMotor
    var deployConfig2 = new TalonFXConfiguration();
    deployConfig2.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    deployConfig2.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    deployConfig2.CurrentLimits.SupplyCurrentLimitEnable = true;
    deployConfig2.CurrentLimits.SupplyCurrentLimit = 30.0;
    deployConfig2.CurrentLimits.SupplyCurrentLowerLimit = 20.0;
    deployConfig2.CurrentLimits.SupplyCurrentLowerTime = 1.0;
    deployConfig2.CurrentLimits.StatorCurrentLimit = 40.0;
    deployConfig2.CurrentLimits.StatorCurrentLimitEnable = true;
    deployLeftMotor.getConfigurator().apply(deployConfig2);
    deployLeftMotor.setControl(new Follower(deployRightMotor.getDeviceID(), MotorAlignmentValue.Opposed));

    // Roller motor — Kraken X44
    var rollerConfig = new TalonFXConfiguration();
    rollerConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    rollerConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    rollerConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    rollerConfig.CurrentLimits.StatorCurrentLimit = 40; // reduced for Kraken X44
    rollerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    rollerConfig.CurrentLimits.SupplyCurrentLimit = 30; // reduced for Kraken X44
    rollerConfig.Slot0.kS = ROLLER_KS;
    rollerConfig.Slot0.kP = ROLLER_KP;
    rollerConfig.Slot0.kD = ROLLER_KD;
    rollerMotor.getConfigurator().apply(rollerConfig);

    deployRightMotor.setPosition(0.0); // assume starting stowed
  }

  @Override
  public void periodic() {
    switch (state) {
      case STOWED:
        deployRightMotor.setControl(motionMagicRequest.withPosition(STOW_POSITION).withSlot(1));
        rollerMotor.setControl(neutralRequest);
        break;
      case DEPLOYING:
        deployRightMotor.setControl(motionMagicRequest.withPosition(DEPLOY_POSITION).withSlot(0));
        rollerMotor.setControl(neutralRequest);
        break;
      case INTAKING:
        deployRightMotor.setControl(motionMagicRequest.withPosition(DEPLOY_POSITION).withSlot(0));
        rollerMotor.setControl(rollerVelocityRequest.withVelocity(INTAKE_RPS));
        break;
      case EJECTING:
        deployRightMotor.setControl(motionMagicRequest.withPosition(DEPLOY_POSITION).withSlot(0));
        rollerMotor.setControl(rollerVelocityRequest.withVelocity(EJECT_RPS));
        break;
      case IDLE_DEPLOYED:
        deployRightMotor.setControl(motionMagicRequest.withPosition(DEPLOY_POSITION).withSlot(0));
        rollerMotor.setControl(neutralRequest);
        break;
    }

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
}
