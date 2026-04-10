package frc.robot.subsystems.leds;

import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.controls.EmptyAnimation;
import com.ctre.phoenix6.controls.LarsonAnimation;
import com.ctre.phoenix6.controls.SingleFadeAnimation;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.LarsonBounceValue;
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

  // ── Strip layout ─────────────────────────────────────────────────────────
  // 4 strips wired in series, in this physical order along the data line:
  //   1. front-right pole — wired bottom→top
  //   2. front-left  pole — wired bottom→top
  //   3. back-left   pole — wired top→bottom
  //   4. back-right  pole — wired bottom→top
  //
  // EDIT THE COUNTS BELOW to match the real LED counts on each strip.
  // Start indices auto-derive from the previous strip, so you only need to
  // touch the *_COUNT values.
  private static final int FRONT_RIGHT_COUNT = 25; // EDIT
  private static final int FRONT_LEFT_COUNT = 20; // EDIT
  private static final int BACK_LEFT_COUNT = 24; // EDIT
  private static final int BACK_RIGHT_COUNT = 21; // EDIT

  private static final int FRONT_RIGHT_START = LED_START;
  private static final int FRONT_RIGHT_END = FRONT_RIGHT_START + FRONT_RIGHT_COUNT - 0;

  private static final int FRONT_LEFT_START = FRONT_RIGHT_END + 0;
  private static final int FRONT_LEFT_END = FRONT_LEFT_START + FRONT_LEFT_COUNT - 0;

  private static final int BACK_LEFT_START = FRONT_LEFT_END + 0;
  private static final int BACK_LEFT_END = BACK_LEFT_START + BACK_LEFT_COUNT - 0;

  private static final int BACK_RIGHT_START = BACK_LEFT_END + 0;
  private static final int BACK_RIGHT_END = BACK_RIGHT_START + BACK_RIGHT_COUNT - 0;

  // Full contiguous range — used by solid/fade controls that paint the whole rig.
  private static final int STRIP_START = FRONT_RIGHT_START;
  private static final int STRIP_END = BACK_RIGHT_END;

  // Larson tuning — applied to all 4 per-strip Larsons.
  private static final int LARSON_SIZE = 4; // EDIT — pocket of light, max 15
  private static final double LARSON_FRAME_RATE = 60.0; // EDIT — Hz, [2, 1000]

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
  private final SolidColor solidAll = new SolidColor(STRIP_START, STRIP_END);
  private final SingleFadeAnimation fadeAll = new SingleFadeAnimation(STRIP_START, STRIP_END);

  // One hardware Larson per strip, each on its own slot. All 4 run in parallel
  // on the CANdle — zero per-frame CAN traffic, smooth at LARSON_FRAME_RATE Hz.
  // Slot assignments: front-right=0, front-left=1, back-left=2, back-right=3.
  private final LarsonAnimation larsonFrontRight =
      new LarsonAnimation(FRONT_RIGHT_START, FRONT_RIGHT_END).withSlot(0);
  private final LarsonAnimation larsonFrontLeft =
      new LarsonAnimation(FRONT_LEFT_START, FRONT_LEFT_END).withSlot(1);
  private final LarsonAnimation larsonBackLeft =
      new LarsonAnimation(BACK_LEFT_START, BACK_LEFT_END).withSlot(2);
  private final LarsonAnimation larsonBackRight =
      new LarsonAnimation(BACK_RIGHT_START, BACK_RIGHT_END).withSlot(3);

  private final TurretSubsystem turret;
  private final BooleanSupplier shootingToggledSupplier;
  private final BooleanSupplier slowModeSupplier;

  // Cached once at construction — reused by checkDisabledHealth() so we don't
  // allocate a new JNI handle every periodic loop.
  private final com.ctre.phoenix6.CANBus rioBus = new com.ctre.phoenix6.CANBus("rio");
  private double lastHealthCheckTimestamp = 0.0;
  private boolean lastHealthResult = true;

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

    if (desired == lastState) return;

    // Leaving DISABLED_ERROR populates 4 slots — slot 0 is handled below by
    // the standard animation→solid clear, but slots 1-3 need explicit clearing
    // or their Larsons keep running underneath whatever the new state paints.
    if (lastState == LEDState.DISABLED_ERROR) {
      candle.setControl(new EmptyAnimation(1));
      candle.setControl(new EmptyAnimation(2));
      candle.setControl(new EmptyAnimation(3));
    }

    // Leaving an animation state for a solid-color state — clear slot 0,
    // otherwise the old animation overlays the new solid color.
    // (Animation→animation transitions are fine: the new animation replaces slot 0.)
    if (isAnimationState(lastState) && !isAnimationState(desired)) {
      candle.setControl(new EmptyAnimation(0));
    }
    lastState = desired;

    switch (desired) {
      case DISABLED_OK:
        // Breathing green — runs on CANdle hardware, smooth animation
        candle.setControl(fadeAll.withColor(GREEN).withFrameRate(80));
        break;
      case DISABLED_ERROR:
        // 4 synced bouncing red Larsons — one per pole, each on its own slot.
        // All run on the CANdle hardware; one CAN frame per strip on entry,
        // then zero RIO/CAN cost while the rig is in this state.
        //        candle.setControl(applyLarsonStyle(larsonFrontRight));
        //        candle.setControl(applyLarsonStyle(larsonFrontLeft));
        //        candle.setControl(applyLarsonStyle(larsonBackLeft));
        //        candle.setControl(applyLarsonStyle(larsonBackRight));
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

  /** Applies the shared Larson style (color, size, bounce, frame rate) to a per-strip Larson. */
  private static LarsonAnimation applyLarsonStyle(LarsonAnimation l) {
    return l.withColor(RED)
        .withSize(LARSON_SIZE)
        .withBounceMode(LarsonBounceValue.Front)
        .withFrameRate(LARSON_FRAME_RATE);
  }

  /**
   * States that send a hardware animation (occupying CANdle animation slot 0, or 0-3 for
   * DISABLED_ERROR). Used so we know when to send {@link EmptyAnimation} on transition out —
   * otherwise the running animation overlays whatever solid color the next state paints.
   */
  private static boolean isAnimationState(LEDState s) {
    return s == LEDState.DISABLED_OK
        || s == LEDState.DISABLED_ERROR
        || s == LEDState.READY_TO_FIRE
        || s == LEDState.FIRING;
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
    if (intakeState == IntakeSubsystem.IntakeState.INTAKING
        || intakeState == IntakeSubsystem.IntakeState.DEPOT_INTAKING) {
      return LEDState.INTAKING;
    }
    if (intakeState == IntakeSubsystem.IntakeState.EJECTING) {
      return LEDState.EJECTING;
    }

    return LEDState.IDLE;
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
