package frc.robot.subsystems.turret; // package frc.robot.subsystems.turret;
//
// import com.ctre.phoenix6.controls.VoltageOut;
// import com.ctre.phoenix6.hardware.TalonFX;
// import edu.wpi.first.math.MathUtil;
// import edu.wpi.first.math.trajectory.TrapezoidProfile;
// import edu.wpi.first.networktables.NetworkTable;
// import edu.wpi.first.networktables.NetworkTableInstance;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
//
// public class TurretSubsystem extends SubsystemBase {
//
//  // Constants
//  private final double kmaxAccelleration = Math.toRadians(60); //    10 rad/s^2 max
//  // TODO: tune
//  private final double kmaxVelocity = Math.toRadians(120); // 10 rad/s max
//  private static final double kP = 0.1; // Proportional
//  private static final double kI = 0.001; // Integral
//  private static final double kD = 0.1; // Derivative
//  private static final double kMaxOutput = 12.0; // volts
//  private static final double kDeadband = 0.5; // degrees
//
//  private final TalonFX turretMotor = new TalonFX(3); // change ID
//  private final NetworkTable limelightTable;
//
//  private double angleBias = 0.0; // multiples of 2π
//
//  private double kmaxlimit = Math.toRadians(360);
//  private double kminlimit = Math.toRadians(-360);
//
//  //
//  private final TrapezoidProfile profile =
//      new TrapezoidProfile(new TrapezoidProfile.Constraints(kmaxVelocity, kmaxAccelleration));
//  private TrapezoidProfile.State setpoint = new TrapezoidProfile.State();
//
//  private final VoltageOut voltageRequest =
//      new VoltageOut(0); // oirejtg9oiw4ehnt98483h34n0y843bn8hb3y4
//
//  public TurretSubsystem() {
//    limelightTable = NetworkTableInstance.getDefault().getTable("limelight-three");
//
//    turretMotor.setPosition(0);
//  }
//
//  @Override
//  public void periodic() {
//    trackTarget();
//    //    turretMotor.set(0.2);
//  }
//
//  private void trackTarget() {
//    double tx = limelightTable.getEntry("tx").getDouble(0.0);
//    double tv = limelightTable.getEntry("tv").getDouble(0.0);
//
//    double currentVelocity = turretMotor.getVelocity().getValueAsDouble() * 2.0 * Math.PI;
//    double currentPosition = turretMotor.getPosition().getValueAsDouble() * 2.0 * Math.PI;
//    // If no target, stop turret
//    if (tv < 1.0) {
//      turretMotor.setControl(voltageRequest.withOutput(0));
//      setpoint = new TrapezoidProfile.State(currentPosition, currentVelocity);
//
//      return;
//    }
//
//    // Deadband
//    if (Math.abs(tx) < kDeadband) {
//      turretMotor.setControl(voltageRequest.withOutput(0));
//      setpoint = new TrapezoidProfile.State(currentPosition, 0.0);
//      return;
//    }
//
//    //    double goalAngle = currentPosition + MathUtil.angleModulus(Math.toRadians(tx));
//    //    goalAngle = MathUtil.clamp(goalAngle, -Math.PI, Math.PI);
//
//    double txRad = tx * (Math.PI / 180.0);
//
//    // shortest path to target
//    //    double wrappedError = MathUtil.angleModulus(txRad);
//    double wrappedError = txRad;
//
//    // propose goal
//    //    double proposedGoal = currentPosition + wrappedError;
//
//    //// enforce soft limits
//    //    double goalAngle = MathUtil.clamp(
//    //            proposedGoal,
//    //            -Math.PI,
//    //            Math.PI
//    //    );
//
//    //    double bestAngle = Double.NaN;
//    //    double bestAngle = proposedGoal;
//
//    double smallestError = Double.POSITIVE_INFINITY;
//
//    // Proposed goal in logical space
//    double proposedGoal = currentPosition + angleBias + txRad;
//
//    // Soft limits in logical space
//    boolean atMin = proposedGoal <= kminlimit;
//    boolean atMax = proposedGoal >= kmaxlimit;
//
//    // If target is beyond limits, unwrap ONCE and STAY THERE
//    if (atMax && txRad > 0) {
//      angleBias -= Math.PI * 0.5;
//    } else if (atMin && txRad < 0) {
//      angleBias += Math.PI * 0.5;
//    }
//
//    // Final goal
//    double bestAngle = currentPosition + angleBias + txRad;
//
//    //
//    //        for (int k = -1; k <= 1; k++) {
//    //          double candidate = proposedGoal + (k * (2.0 * Math.PI));
//    //
//    //          if ((candidate >= -(Math.PI * 40)) && (candidate <= (Math.PI * 40))) {
//    //            double error = Math.abs(candidate - currentPosition);
//    //            if (error < smallestError) {
//    //              smallestError = error;
//    //              bestAngle = candidate;
//    //            }
//    //          }
//    //        }
//    bestAngle = MathUtil.clamp(proposedGoal, kminlimit, kmaxlimit);
//    System.out.println(
//        bestAngle + "Best Angle" + currentPosition + "error" + tx + "tx" + angleBias + "bias");
//
//    //    if (Double.isNaN(bestAngle)) {
//
//    //    }
//
//    TrapezoidProfile.State goalState = new TrapezoidProfile.State(bestAngle, 0.0);
//
//    setpoint = profile.calculate(0.02, setpoint, goalState);
//
//    //    double posError = setpoint.position - currentPosition;
//    double posError = bestAngle - currentPosition;
//    double velError = -currentVelocity;
//    double outputVolts = kP * posError + kD * velError;
//    //    System.out.println(outputVolts + "Output Volts");
//    //            + kD * velError;
//
//    //    // Clamp voltage
//    outputVolts = MathUtil.clamp(outputVolts, -kMaxOutput, kMaxOutput);
//
//    turretMotor.setVoltage(outputVolts);
//    //    turretMotor.getPosition()
//  }
// }
