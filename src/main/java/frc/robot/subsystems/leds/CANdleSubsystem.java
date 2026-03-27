package frc.robot.subsystems.leds;

import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.controls.StrobeAnimation;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.RGBWColor;
import com.ctre.phoenix6.signals.StripTypeValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooting.HoodSubsystem;
import frc.robot.subsystems.shooting.ShooterSubsystem;
import frc.robot.subsystems.shooting.TurretSubsystem;

public class CANdleSubsystem extends SubsystemBase {

  private static final int CANDLE_ID = 10; // tune
  private static final int STRIP_LENGTH = 300; // tune —
  private static final int LED_START = 8; // first external led cuz first 7 is the on candle ones
  private static final int LED_END = LED_START + STRIP_LENGTH - 1;

  private static final RGBWColor OFF = new RGBWColor(0, 0, 0);
  private static final RGBWColor GREEN = new RGBWColor(0, 255, 0);
  private static final RGBWColor YELLOW = new RGBWColor(255, 180, 0);
  private static final RGBWColor RED = new RGBWColor(255, 0, 0);
  private static final RGBWColor BLUE = new RGBWColor(0, 0, 255);
  private static final RGBWColor ORANGE = new RGBWColor(255, 80, 0);
  private static final RGBWColor PURPLE = new RGBWColor(128, 0, 255);
  private static final RGBWColor WHITE = new RGBWColor(255, 255, 255);

  private final CANdle candle;
  private final SolidColor solidRequest = new SolidColor(LED_START, LED_END);
  private final StrobeAnimation strobeRequest = new StrobeAnimation(LED_START, LED_END);

  private LEDState lastState = null;

  private enum LEDState {
    OFF,
    IDLE,
    SPINNING_UP,
    AT_SPEED,
    READY_TO_FIRE,
    FIRING,
    PASSING,
    INTAKING,
    EJECTING
  }

  private final TurretSubsystem turret;

  public CANdleSubsystem(TurretSubsystem turret) {
    this.turret = turret;
    candle = new CANdle(CANDLE_ID);

    var config = new CANdleConfiguration();
    config.LED.StripType = StripTypeValue.GRB;
    config.LED.BrightnessScalar = 1.0;
    candle.getConfigurator().apply(config);

    // Start off
    candle.setControl(solidRequest.withColor(OFF));
  }

  @Override
  public void periodic() {
    LEDState desired = determineState();

    if (desired == lastState) return;
    lastState = desired;

    switch (desired) {
      case OFF:
      case IDLE:
        candle.setControl(solidRequest.withColor(OFF));
        break;
      case SPINNING_UP:
        candle.setControl(solidRequest.withColor(YELLOW));
        break;
      case AT_SPEED:
        candle.setControl(solidRequest.withColor(GREEN));
        break;
      case READY_TO_FIRE:
        candle.setControl(strobeRequest.withColor(GREEN).withFrameRate(1));
        break;
      case FIRING:
        candle.setControl(strobeRequest.withColor(WHITE).withFrameRate(1));
        break;
      case PASSING:
        candle.setControl(solidRequest.withColor(ORANGE));
        break;
      case INTAKING:
        candle.setControl(solidRequest.withColor(BLUE));
        break;
      case EJECTING:
        candle.setControl(solidRequest.withColor(PURPLE));
        break;
    }
  }

  private LEDState determineState() {
    ShooterSubsystem shooter = ShooterSubsystem.getInstance();
    HoodSubsystem hood = HoodSubsystem.getInstance();
    IntakeSubsystem intake = IntakeSubsystem.getInstance();

    ShooterSubsystem.ShooterState shooterState = shooter.getState();
    IntakeSubsystem.IntakeState intakeState = intake.getState();

    if (shooterState == ShooterSubsystem.ShooterState.RAPID_FIRE
        || shooterState == ShooterSubsystem.ShooterState.RAPID_FIRE_ACCURATE) {
      return LEDState.FIRING;
    }

    if (shooterState == ShooterSubsystem.ShooterState.PASSING) {
      return LEDState.PASSING;
    }

    if (shooterState == ShooterSubsystem.ShooterState.SPIN_UP) {
      boolean atSpeed = shooter.isAtGoalSpeed();
      boolean hoodReady = hood.atTarget();
      boolean onTarget = turret.isOnTarget();

      if (atSpeed && hoodReady && onTarget) {
        return LEDState.READY_TO_FIRE;
      } else if (atSpeed) {
        return LEDState.AT_SPEED;
      } else {
        return LEDState.SPINNING_UP;
      }
    }

    if (intakeState == IntakeSubsystem.IntakeState.INTAKING) {
      return LEDState.INTAKING;
    }
    if (intakeState == IntakeSubsystem.IntakeState.EJECTING) {
      return LEDState.EJECTING;
    }

    return LEDState.IDLE;
  }
}
