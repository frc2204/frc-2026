package frc.robot.subsystems.leds;

import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.RGBWColor;
import com.ctre.phoenix6.signals.StripTypeValue;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooting.HoodSubsystem;
import frc.robot.subsystems.shooting.ShooterSubsystem;
import frc.robot.subsystems.shooting.TurretSubsystem;
import java.util.function.BooleanSupplier;

public class CANdleSubsystem extends SubsystemBase {

  private static final int CANDLE_ID = 10; // tune
  private static final int LED_START = 8; // first external LED (0-7 are onboard)

  // Right strip: LEDs 8-33 (indices 8 to 8+25)
  private static final int RIGHT_START = LED_START;
  private static final int RIGHT_END = LED_START + 25; // 26 LEDs (0-25)

  // Left strip: LEDs 34-52 (indices 26-44, continues from right)
  private static final int LEFT_START = RIGHT_END + 1;
  private static final int LEFT_END = LED_START + 44; // 19 LEDs (26-44)

  private static final double BREATHE_PERIOD = 1.5; // seconds per full breathe cycle

  private static final RGBWColor OFF = new RGBWColor(0, 0, 0);
  private static final RGBWColor GREEN = new RGBWColor(0, 255, 0);
  private static final RGBWColor YELLOW = new RGBWColor(255, 180, 0);
  private static final RGBWColor RED = new RGBWColor(255, 0, 0);
  private static final RGBWColor BLUE = new RGBWColor(0, 0, 255);
  private static final RGBWColor ORANGE = new RGBWColor(255, 80, 0);
  private static final RGBWColor PURPLE = new RGBWColor(128, 0, 255);
  private static final RGBWColor WHITE = new RGBWColor(255, 255, 255);
  private static final RGBWColor CYAN = new RGBWColor(0, 255, 255);

  private final CANdle candle;
  private final SolidColor solidAll = new SolidColor(RIGHT_START, LEFT_END);

  private final TurretSubsystem turret;
  private final BooleanSupplier shootingToggledSupplier;
  private final BooleanSupplier slowModeSupplier;

  private LEDState lastState = null;

  private enum LEDState {
    DISABLED_OK,
    DISABLED_ERROR,
    OFF,
    IDLE,
    SHOOTING_TOGGLED,
    SLOW_MODE,
    SPINNING_UP,
    AT_SPEED,
    READY_TO_FIRE,
    FIRING,
    PASSING,
    INTAKING,
    EJECTING
  }

  public CANdleSubsystem(
      TurretSubsystem turret,
      BooleanSupplier shootingToggledSupplier,
      BooleanSupplier slowModeSupplier) {
    this.turret = turret;
    this.shootingToggledSupplier = shootingToggledSupplier;
    this.slowModeSupplier = slowModeSupplier;
    candle = new CANdle(CANDLE_ID, frc.robot.generated.TunerConstants.kCANBus);

    var config = new CANdleConfiguration();
    config.LED.StripType = StripTypeValue.RGB;
    config.LED.BrightnessScalar = 1.0;
    candle.getConfigurator().apply(config);

    candle.setControl(solidAll.withColor(OFF));
  }

  @Override
  public void periodic() {
    LEDState desired = determineState();
    boolean breathing = desired == LEDState.READY_TO_FIRE || desired == LEDState.FIRING;

    boolean disabled = desired == LEDState.DISABLED_OK || desired == LEDState.DISABLED_ERROR;

    // Always update for breathing and disabled states. Solid enabled states only on change.
    if (!breathing && !disabled && desired == lastState) return;
    lastState = desired;

    switch (desired) {
      case DISABLED_OK:
        candle.setControl(solidAll.withColor(GREEN));
        break;
      case DISABLED_ERROR:
        candle.setControl(solidAll.withColor(RED));
        break;
      case OFF:
      case IDLE:
        candle.setControl(solidAll.withColor(OFF));
        break;
      case SHOOTING_TOGGLED:
        candle.setControl(solidAll.withColor(ORANGE));
        break;
      case SLOW_MODE:
        candle.setControl(solidAll.withColor(CYAN));
        break;
      case SPINNING_UP:
        candle.setControl(solidAll.withColor(YELLOW));
        break;
      case AT_SPEED:
        candle.setControl(solidAll.withColor(GREEN));
        break;
      case READY_TO_FIRE:
        candle.setControl(solidAll.withColor(breathe(0, 255, 0)));
        break;
      case FIRING:
        candle.setControl(solidAll.withColor(breathe(255, 255, 255)));
        break;
      case PASSING:
        candle.setControl(solidAll.withColor(ORANGE));
        break;
      case INTAKING:
        candle.setControl(solidAll.withColor(BLUE));
        break;
      case EJECTING:
        candle.setControl(solidAll.withColor(PURPLE));
        break;
    }
  }

  /** Returns a scaled version of the color using a sine wave for breathing effect. */
  private RGBWColor breathe(int r, int g, int b) {
    double t = Timer.getFPGATimestamp();
    double brightness = (Math.sin(2.0 * Math.PI * t / BREATHE_PERIOD) + 1.0) / 2.0;
    // Clamp minimum so it doesn't fully turn off
    brightness = 0.15 + brightness * 0.85;
    return new RGBWColor((int) (r * brightness), (int) (g * brightness), (int) (b * brightness));
  }

  private LEDState determineState() {
    // ── DISABLED: diagnostic status ──
    if (DriverStation.isDisabled()) {
      boolean healthy = checkDisabledHealth();
      return healthy ? LEDState.DISABLED_OK : LEDState.DISABLED_ERROR;
    }

    // ── ENABLED: game state ──
    ShooterSubsystem shooter = ShooterSubsystem.getInstance();
    HoodSubsystem hood = HoodSubsystem.getInstance();
    IntakeSubsystem intake = IntakeSubsystem.getInstance();

    ShooterSubsystem.ShooterState shooterState = shooter.getState();
    IntakeSubsystem.IntakeState intakeState = intake.getState();

    // Shooting states (highest priority)
    if (shooterState == ShooterSubsystem.ShooterState.RAPID_FIRE
        || shooterState == ShooterSubsystem.ShooterState.RAPID_FIRE_ACCURATE
        || shooterState == ShooterSubsystem.ShooterState.OVERRIDE) {
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

    // Intake states
    if (intakeState == IntakeSubsystem.IntakeState.INTAKING) {
      return LEDState.INTAKING;
    }
    if (intakeState == IntakeSubsystem.IntakeState.EJECTING) {
      return LEDState.EJECTING;
    }

    // Driver mode indicators
    if (shootingToggledSupplier.getAsBoolean()) {
      return LEDState.SHOOTING_TOGGLED;
    }
    if (slowModeSupplier.getAsBoolean()) {
      return LEDState.SLOW_MODE;
    }

    return LEDState.IDLE;
  }

  /** Checks system health while disabled. */
  private boolean checkDisabledHealth() {
    boolean joystick = DriverStation.isJoystickConnected(0);
    boolean battery = RobotController.getBatteryVoltage() >= 11.5;
    boolean candleOk = candle.isConnected();

    com.ctre.phoenix6.CANBus canivore = frc.robot.generated.TunerConstants.kCANBus;
    com.ctre.phoenix6.CANBus rio = new com.ctre.phoenix6.CANBus("rio");
    boolean canivoreOk = canivore.getStatus().Status.isOK();
    boolean rioOk = rio.getStatus().Status.isOK();

    org.littletonrobotics.junction.Logger.recordOutput("LEDs/JoystickConnected", joystick);
    org.littletonrobotics.junction.Logger.recordOutput("LEDs/BatteryOK", battery);
    org.littletonrobotics.junction.Logger.recordOutput("LEDs/CANdleConnected", candleOk);
    org.littletonrobotics.junction.Logger.recordOutput("LEDs/CANivoreOK", canivoreOk);
    org.littletonrobotics.junction.Logger.recordOutput("LEDs/RioBusOK", rioOk);

    return joystick && battery && canivoreOk && rioOk;
  }
}
