package frc.robot.subsystems.leds;

import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.controls.SingleFadeAnimation;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.RGBWColor;
import com.ctre.phoenix6.signals.StripTypeValue;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooting.HoodSubsystem;
import frc.robot.subsystems.shooting.ShooterSubsystem;
import frc.robot.subsystems.shooting.TurretSubsystem;
import java.util.function.BooleanSupplier;

public class CANdleSubsystem extends SubsystemBase {

  private static final int CANDLE_ID = 10; // tune
  private static final int LED_START = 8; // first external LED (0-7 are onboard)

  // Right strip: LEDs 8-32 (25 LEDs)
  private static final int RIGHT_START = LED_START;
  private static final int RIGHT_END = LED_START + 24; // inclusive — 25 LEDs

  // Left strip: LEDs 33-55 (23 LEDs, continues from right)
  private static final int LEFT_START = RIGHT_END + 1;
  private static final int LEFT_END = LEFT_START + 22; // inclusive — 23 LEDs

  // Disabled-error Larson cycle.
  // Phase 1: right bottom→top   (RIGHT_LEN steps)
  // Phase 2: left  bottom→top   (LEFT_LEN  steps)
  // Phase 3: left  top→bottom   (LEFT_LEN  steps)
  // ↳ loops back to phase 1 (which the user described as "right bottom to top" again)
  // If you want right to also bounce back (true 4-phase Larson), add a 4th phase
  // that decrements RIGHT_END→RIGHT_START and bump LARSON_TOTAL_STEPS by RIGHT_LEN.
  private static final int RIGHT_LEN = RIGHT_END - RIGHT_START + 1;
  private static final int LEFT_LEN = LEFT_END - LEFT_START + 1;
  private static final int LARSON_FRAMES_PER_STEP = 1; // bump for slower
  private static final int LARSON_TOTAL_STEPS = RIGHT_LEN + LEFT_LEN + LEFT_LEN;

  private static final RGBWColor OFF = new RGBWColor(0, 0, 0);
  private static final RGBWColor GREEN = new RGBWColor(0, 255, 0);
  private static final RGBWColor YELLOW = new RGBWColor(255, 180, 0);
  private static final RGBWColor RED = new RGBWColor(0xFF, 0x12, 0x0D); // #FF120D
  private static final RGBWColor PINK = new RGBWColor(255, 105, 180);
  private static final RGBWColor ORANGE = new RGBWColor(255, 80, 0);
  private static final RGBWColor PURPLE = new RGBWColor(128, 0, 255);
  private static final RGBWColor WHITE = new RGBWColor(255, 255, 255);
  private static final RGBWColor CYAN = new RGBWColor(0, 255, 255);

  private final CANdle candle;
  private final SolidColor solidAll = new SolidColor(RIGHT_START, LEFT_END);
  private final SingleFadeAnimation fadeAll = new SingleFadeAnimation(RIGHT_START, LEFT_END);

  private final TurretSubsystem turret;
  private final BooleanSupplier shootingToggledSupplier;
  private final BooleanSupplier slowModeSupplier;

  // Cached once at construction — reused by checkDisabledHealth() so we don't
  // allocate a new JNI handle every periodic loop.
  private final com.ctre.phoenix6.CANBus rioBus = new com.ctre.phoenix6.CANBus("rio");
  private double lastHealthCheckTimestamp = 0.0;
  private boolean lastHealthResult = true;

  // Disabled-error Larson scanner state.
  private final SolidColor larsonBackground = new SolidColor(RIGHT_START, LEFT_END);
  private int larsonFrame = 0;

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

    // DISABLED_ERROR runs a manually-stepped Larson scanner, so it needs to update
    // every frame instead of only on state changes.
    if (desired == LEDState.DISABLED_ERROR) {
      if (lastState != LEDState.DISABLED_ERROR) {
        lastState = LEDState.DISABLED_ERROR;
        larsonFrame = 0;
      }
      updateErrorLarson();
      return;
    }

    if (desired == lastState) return;
    lastState = desired;

    switch (desired) {
      case DISABLED_OK:
        // Breathing green — runs on CANdle hardware, smooth animation
        candle.setControl(fadeAll.withColor(GREEN).withFrameRate(80));
        break;
      case DISABLED_ERROR:
        // Handled above, but keep a fallback so the switch is exhaustive.
        candle.setControl(solidAll.withColor(RED));
        break;
      case OFF:
      case IDLE:
        candle.setControl(solidAll.withColor(OFF));
        break;
      case SLOW_MODE:
        candle.setControl(solidAll.withColor(CYAN));
        break;
      case SHOOTING_TOGGLED:
        candle.setControl(solidAll.withColor(ORANGE));
        break;
      case SPINNING_UP:
        candle.setControl(solidAll.withColor(YELLOW));
        break;
      case AT_SPEED:
        candle.setControl(solidAll.withColor(GREEN));
        break;
      case READY_TO_FIRE:
        candle.setControl(fadeAll.withColor(GREEN).withFrameRate(80));
        break;
      case FIRING:
        candle.setControl(fadeAll.withColor(WHITE).withFrameRate(80));
        break;
      case PASSING:
        candle.setControl(solidAll.withColor(ORANGE));
        break;
      case INTAKING:
        candle.setControl(solidAll.withColor(PINK));
        break;
      case EJECTING:
        candle.setControl(solidAll.withColor(PURPLE));
        break;
    }
  }

  private LEDState determineState() {
    // ── DISABLED: diagnostic status ──
    if (DriverStation.isDisabled()) {
      boolean healthy = checkDisabledHealth();
      return healthy ? LEDState.DISABLED_OK : LEDState.DISABLED_ERROR;
    }

    // Slow mode — highest enabled priority
    if (slowModeSupplier.getAsBoolean()) {
      return LEDState.SLOW_MODE;
    }

    // shooting toggled second priority
    if (shootingToggledSupplier.getAsBoolean()) {
      return LEDState.SHOOTING_TOGGLED;
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

    return LEDState.IDLE;
  }

  /**
   * Steps the disabled-error Larson scanner one frame. Called every periodic while in
   * DISABLED_ERROR. Paints the strip OFF and lights a single head LED RED at the position derived
   * from {@link #larsonFrame}.
   */
  private void updateErrorLarson() {
    int step = (larsonFrame / LARSON_FRAMES_PER_STEP) % LARSON_TOTAL_STEPS;
    larsonFrame++;

    int headIdx;
    if (step < RIGHT_LEN) {
      // Phase 1: right strip bottom → top
      headIdx = RIGHT_START + step;
    } else if (step < RIGHT_LEN + LEFT_LEN) {
      // Phase 2: left strip bottom → top
      headIdx = LEFT_START + (step - RIGHT_LEN);
    } else {
      // Phase 3: left strip top → bottom
      headIdx = LEFT_END - (step - RIGHT_LEN - LEFT_LEN);
    }

    // Background OFF, then head RED. Two setControl calls per loop is cheap.
    candle.setControl(larsonBackground.withColor(OFF));
    candle.setControl(new SolidColor(headIdx, headIdx).withColor(RED));
  }

  /** Checks system health while disabled. Throttled to 1 Hz to keep loop time bounded. */
  private boolean checkDisabledHealth() {
    double now = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
    if (now - lastHealthCheckTimestamp < 1.0) {
      return lastHealthResult;
    }
    lastHealthCheckTimestamp = now;

    boolean joystick = DriverStation.isJoystickConnected(0);
    boolean battery = RobotController.getBatteryVoltage() >= 11.5;
    boolean candleOk = candle.isConnected();

    // Skip CAN-bus status reads in sim — neither bus actually exists and the
    // JNI calls can block at 50 Hz, causing loop overruns + comms drops.
    boolean canivoreOk;
    boolean rioOk;
    if (edu.wpi.first.wpilibj.RobotBase.isReal()) {
      canivoreOk = frc.robot.generated.TunerConstants.kCANBus.getStatus().Status.isOK();
      rioOk = rioBus.getStatus().Status.isOK();
    } else {
      canivoreOk = true;
      rioOk = true;
    }

    org.littletonrobotics.junction.Logger.recordOutput("LEDs/JoystickConnected", joystick);
    org.littletonrobotics.junction.Logger.recordOutput("LEDs/BatteryOK", battery);
    org.littletonrobotics.junction.Logger.recordOutput("LEDs/CANdleConnected", candleOk);
    org.littletonrobotics.junction.Logger.recordOutput("LEDs/CANivoreOK", canivoreOk);
    org.littletonrobotics.junction.Logger.recordOutput("LEDs/RioBusOK", rioOk);

    lastHealthResult = joystick && battery && canivoreOk && rioOk;
    return lastHealthResult;
  }
}
