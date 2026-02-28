package frc.robot.subsystems.shooting;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.handoff.HandoffSubsystem;
import org.littletonrobotics.junction.Logger;

public class ShooterSubsystem extends SubsystemBase {
  private static final ShooterSubsystem INSTANCE = new ShooterSubsystem();

  public static ShooterSubsystem getInstance() {
    return INSTANCE;
  }

  public enum ShooterState {
    IDLE,
    SPIN_UP,
    RAPID_FIRE,
    RAPID_FIRE_ACCURATE,
    OVERRIDE,
    PASSING,
    EMPTY
  }

  private ShooterState state = ShooterState.IDLE;

  // ── TUNING GUIDE ──────────────────────────────────────────────────────────
  // All PID values appear on SmartDashboard under "Shooter/PID/...".
  // Changes take effect immediately — no redeploy needed.
  //
  // Slot 0 (Steady) — holds flywheel at target RPM when no ball is passing through.
  //   kS: static friction voltage. Increase if flywheel won't spin at low RPM.
  //   kV: velocity feedforward (volts per RPS). Set so kV * targetRPS ≈ voltage to
  //       hold that speed with no load. Measure by plotting voltage vs free-spin RPM.
  //   kA: acceleration feedforward. Usually very small for flywheels; leave near 0.
  //   kP_STEADY: proportional gain. Start low (~0.1), increase until error is small
  //       without oscillation. If flywheel oscillates around the setpoint, lower kP.
  //   kD_STEADY: derivative gain. Dampens overshoot. Increase if kP causes ringing.
  //       If the system feels sluggish, lower kD.
  //
  // Slot 1 (Recovery) — kicks in when a ball passes through and RPM drops.
  //   kP_RECOVERY: higher than steady to recover RPM aggressively after a shot.
  //       If recovery overshoots, lower this. If recovery is too slow, raise it.
  //   kD_RECOVERY: dampens the aggressive recovery. Raise if recovery oscillates.
  //
  // WORKFLOW:
  //   1. Tune kS and kV first with kP/kD at 0 — flywheel should roughly hold speed.
  //   2. Add kP_STEADY until error is <50 RPM at steady state.
  //   3. Add kD_STEADY if there's oscillation.
  //   4. Fire a ball — watch recovery. Raise kP_RECOVERY if slow, kD_RECOVERY if it rings.
  // ──────────────────────────────────────────────────────────────────────────

  // holding speed TODO: add final back later
  static double kS = 0.16;
  static double kV = 0.104;
  static double kA = 0.001;
  static double kP_STEADY = 0.25;
  static double kD_STEADY = 0.02;

  // getting back to speed after shot
  static double kP_RECOVERY = 0.35;
  static double kD_RECOVERY = 0.04;

  // rpm error to switch slots
  static final double RECOVERY_THRESHOLD_RPS = 100.0 / 60.0; // low since flywheel drop is small

  // delay from handoff beam break
  private static final double BEAM_BREAK_DELAY_SECONDS = 0.15; // tune: travel time to flywheel
  private double beamBreakTriggerTime = -1.0;
  private boolean beamBreakPredicting = false;

  private final TalonFX shooterMotor = new TalonFX(23); // change ID
  private final TalonFX shooterMotor2 = new TalonFX(24); // change ID
  private static final double GEAR_RATIO = 1.0;

  // config
  private final double overspinFactor = 1.07;
  private final double rpmRapidFireTolerance = 150;
  private final double rpmAccurateTolerance = 50;
  private final double passingTolerance = 500; // tune

  private final SlewRateLimiter spinUpRamp = new SlewRateLimiter(40); // rps

  private final VelocityVoltage velocityRequest = new VelocityVoltage(0);
  private final VoltageOut voltageRequest = new VoltageOut(0);

  private static final InterpolatingDoubleTreeMap shotFlywheelSpeedMap =
      new InterpolatingDoubleTreeMap();

  static {
    // example — replace with real data
    // distance (m) -> rpm
    shotFlywheelSpeedMap.put(Units.inchesToMeters(30.0 + 27 / 2 + 23.25 + 3.5), 2450.0);
    shotFlywheelSpeedMap.put(Units.inchesToMeters(40.0 + 27 / 2 + 23.25 + 3.5), 2550.0);
    shotFlywheelSpeedMap.put(Units.inchesToMeters(50.0 + 27 / 2 + 23.25 + 3.5), 2600.0);
    shotFlywheelSpeedMap.put(Units.inchesToMeters(60.0 + 27 / 2 + 23.25 + 3.5), 2600.0);
    shotFlywheelSpeedMap.put(Units.inchesToMeters(70.0 + 27 / 2 + 23.25 + 3.5), 2750.0);
    shotFlywheelSpeedMap.put(Units.inchesToMeters(80.0 + 27 / 2 + 23.25 + 3.5), 2800.0);
    shotFlywheelSpeedMap.put(Units.inchesToMeters(130), 2850.0);
    shotFlywheelSpeedMap.put(Units.inchesToMeters(140), 3000.0);
    shotFlywheelSpeedMap.put(Units.inchesToMeters(150), 3000.0);
    shotFlywheelSpeedMap.put(Units.inchesToMeters(160), 3100.0);
    shotFlywheelSpeedMap.put(Units.inchesToMeters(170), 3200.0);
  }

  private ShooterSubsystem() {
    // publish PID values to SmartDashboard for live tuning
    SmartDashboard.putNumber("Target Speed", 0.0);
    SmartDashboard.putNumber("Shooter/PID/kS", kS);
    SmartDashboard.putNumber("Shooter/PID/kV", kV);
    SmartDashboard.putNumber("Shooter/PID/kA", kA);
    SmartDashboard.putNumber("Shooter/PID/kP_Steady", kP_STEADY);
    SmartDashboard.putNumber("Shooter/PID/kD_Steady", kD_STEADY);
    SmartDashboard.putNumber("Shooter/PID/kP_Recovery", kP_RECOVERY);
    SmartDashboard.putNumber("Shooter/PID/kD_Recovery", kD_RECOVERY);

    var config = new TalonFXConfiguration();
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    // 70a burst for 1.5s drop to 35a sustained
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = 70.0;
    config.CurrentLimits.SupplyCurrentLowerLimit = 35.0;
    config.CurrentLimits.SupplyCurrentLowerTime = 1.5;
    config.CurrentLimits.StatorCurrentLimit = 120.0;
    config.CurrentLimits.StatorCurrentLimitEnable = true;

    config.Slot0.kS = kS;
    config.Slot0.kV = kV;
    config.Slot0.kA = kA;
    config.Slot0.kP = kP_STEADY;
    config.Slot0.kI = 0.0;
    config.Slot0.kD = kD_STEADY;

    config.Slot1.kS = kS;
    config.Slot1.kV = kV;
    config.Slot1.kA = kA;
    config.Slot1.kP = kP_RECOVERY;
    config.Slot1.kI = 0.0;
    config.Slot1.kD = kD_RECOVERY;

    shooterMotor.getConfigurator().apply(config);

    // follower
    var config2 = new TalonFXConfiguration();
    config2.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    config2.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    config2.CurrentLimits.SupplyCurrentLimitEnable = true;
    config2.CurrentLimits.SupplyCurrentLimit = 70.0;
    config2.CurrentLimits.SupplyCurrentLowerLimit = 35.0;
    config2.CurrentLimits.SupplyCurrentLowerTime = 1.5;
    config2.CurrentLimits.StatorCurrentLimit = 120.0;
    config2.CurrentLimits.StatorCurrentLimitEnable = true;
    shooterMotor2.getConfigurator().apply(config2);
    shooterMotor2.setControl(new Follower(shooterMotor.getDeviceID(), MotorAlignmentValue.Opposed));
  }

  @Override
  public void periodic() {
    updatePIDFromDashboard();

    double distance = getTargetDistance();

    double baseRPM = shotFlywheelSpeedMap.get(distance);
    //    double baseRPM = SmartDashboard.getNumber("Target Speed", 0.0);
    double targetRPM = baseRPM * overspinFactor;
    double currentRPM = getVelocityRevPerSec() * 60.0;

    double targetRPS = targetRPM / 60.0;
    double currentRPS = currentRPM / 60.0;

    double now = Timer.getFPGATimestamp();
    boolean handoffBeamBroken = HandoffSubsystem.getInstance().isBeamBroken();

    if (handoffBeamBroken && beamBreakTriggerTime < 0) {
      beamBreakTriggerTime = now;
    } else if (!handoffBeamBroken) {
      beamBreakTriggerTime = -1.0;
    }

    beamBreakPredicting =
        beamBreakTriggerTime > 0 && (now - beamBreakTriggerTime) >= BEAM_BREAK_DELAY_SECONDS;

    int activeSlot = 0;

    switch (state) {
      case SPIN_UP:
        double rampedRPS = spinUpRamp.calculate(targetRPS);
        shooterMotor.setControl(velocityRequest.withVelocity(rampedRPS).withSlot(0));
        break;
      case RAPID_FIRE:
      case OVERRIDE:
      case RAPID_FIRE_ACCURATE:
        // slot 1 if beam break or rpm error exceeds threshold
        double error = Math.abs(targetRPS - currentRPS);
        boolean useRecovery = beamBreakPredicting || error > RECOVERY_THRESHOLD_RPS;
        activeSlot = useRecovery ? 1 : 0;
        shooterMotor.setControl(velocityRequest.withVelocity(targetRPS).withSlot(activeSlot));
        break;
      case PASSING:
        shooterMotor.setControl(voltageRequest.withOutput(10.0)); // full voltage for passing
        break;
      case IDLE:
      case EMPTY:
        shooterMotor.setControl(voltageRequest.withOutput(0.0));
        spinUpRamp.reset(0.0);
        beamBreakTriggerTime = -1.0;
        beamBreakPredicting = false;
        break;
    }

    Logger.recordOutput("Shooter/TargetRPM", targetRPM);
    Logger.recordOutput("Shooter/CurrentRPM", currentRPM);
    Logger.recordOutput("Shooter/ErrorRPM", targetRPM - currentRPM);
    Logger.recordOutput("Shooter/TargetRPS", targetRPS);
    Logger.recordOutput("Shooter/CurrentRPS", currentRPS);
    Logger.recordOutput("Shooter/ActiveSlot", activeSlot);
    Logger.recordOutput("Shooter/State", state.name());
    Logger.recordOutput("Shooter/AtGoalSpeed", isAtGoalSpeed());
    Logger.recordOutput("Shooter/BeamBreakPredicting", beamBreakPredicting);
    Logger.recordOutput("Shooter/DistanceM", distance);
    Logger.recordOutput(
        "Shooter/SupplyCurrent", shooterMotor.getSupplyCurrent().getValueAsDouble());
    Logger.recordOutput(
        "Shooter/StatorCurrent", shooterMotor.getStatorCurrent().getValueAsDouble());
  }

  double getVelocityRevPerSec() {
    double motorRPS = shooterMotor.getVelocity().getValueAsDouble();
    return (motorRPS / GEAR_RATIO);
  }

  public void setState(ShooterState newState) {
    if (newState == ShooterState.SPIN_UP && state == ShooterState.IDLE) {
      spinUpRamp.reset(getVelocityRevPerSec());
    }
    state = newState;
  }

  public ShooterState getState() {
    return state;
  }

  public boolean isAtGoalSpeed() {
    double distance = getTargetDistance();
    double targetRPM = shotFlywheelSpeedMap.get(distance) * overspinFactor;
    //    double targetRPM = SmartDashboard.getNumber("Target Speed", 0.0) * overspinFactor;
    double currentRPM = getVelocityRevPerSec() * 60.0;
    if (state == ShooterState.PASSING) {
      return Math.abs(currentRPM - targetRPM) <= passingTolerance;
    }
    return Math.abs(currentRPM - targetRPM) <= rpmRapidFireTolerance;
  }

  public boolean isAtGoalSpeedAccurate() {
    double distance = getTargetDistance();
    double targetRPM = shotFlywheelSpeedMap.get(distance) * overspinFactor;
    //    double targetRPM = SmartDashboard.getNumber("Target Speed", 0.0) * overspinFactor;
    double currentRPM = getVelocityRevPerSec() * 60.0;
    return Math.abs(currentRPM - targetRPM) <= rpmAccurateTolerance;
  }

  private double targetDistance = 0.0;

  public void setTargetDistance(double distance) {
    targetDistance = distance;
  }

  private double getTargetDistance() {
    return targetDistance;
  }

  public void onEnable() {
    spinUpRamp.reset(getVelocityRevPerSec());
  }

  private void updatePIDFromDashboard() {
    double newKS = SmartDashboard.getNumber("Shooter/PID/kS", kS);
    double newKV = SmartDashboard.getNumber("Shooter/PID/kV", kV);
    double newKA = SmartDashboard.getNumber("Shooter/PID/kA", kA);
    double newKP_Steady = SmartDashboard.getNumber("Shooter/PID/kP_Steady", kP_STEADY);
    double newKD_Steady = SmartDashboard.getNumber("Shooter/PID/kD_Steady", kD_STEADY);
    double newKP_Recovery = SmartDashboard.getNumber("Shooter/PID/kP_Recovery", kP_RECOVERY);
    double newKD_Recovery = SmartDashboard.getNumber("Shooter/PID/kD_Recovery", kD_RECOVERY);

    boolean changed =
        newKS != kS
            || newKV != kV
            || newKA != kA
            || newKP_Steady != kP_STEADY
            || newKD_Steady != kD_STEADY
            || newKP_Recovery != kP_RECOVERY
            || newKD_Recovery != kD_RECOVERY;

    if (!changed) return;

    kS = newKS;
    kV = newKV;
    kA = newKA;
    kP_STEADY = newKP_Steady;
    kD_STEADY = newKD_Steady;
    kP_RECOVERY = newKP_Recovery;
    kD_RECOVERY = newKD_Recovery;

    var config = new TalonFXConfiguration();
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = 70.0;
    config.CurrentLimits.SupplyCurrentLowerLimit = 35.0;
    config.CurrentLimits.SupplyCurrentLowerTime = 1.5;
    config.CurrentLimits.StatorCurrentLimit = 120.0;
    config.CurrentLimits.StatorCurrentLimitEnable = true;

    config.Slot0.kS = kS;
    config.Slot0.kV = kV;
    config.Slot0.kA = kA;
    config.Slot0.kP = kP_STEADY;
    config.Slot0.kI = 0.0;
    config.Slot0.kD = kD_STEADY;

    config.Slot1.kS = kS;
    config.Slot1.kV = kV;
    config.Slot1.kA = kA;
    config.Slot1.kP = kP_RECOVERY;
    config.Slot1.kI = 0.0;
    config.Slot1.kD = kD_RECOVERY;

    shooterMotor.getConfigurator().apply(config);
  }
}
