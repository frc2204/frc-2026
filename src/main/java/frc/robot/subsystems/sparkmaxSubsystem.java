// package frc.robot.subsystems;
//
// import com.revrobotics.PersistMode;
// import com.revrobotics.ResetMode;
// import com.revrobotics.spark.SparkLowLevel;
// import com.revrobotics.spark.SparkMax;
// import com.revrobotics.spark.config.SparkBaseConfig;
// import com.revrobotics.spark.config.SparkMaxConfig;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.subsystems.shooting.ShooterSubsystem;
//
// public class sparkmaxSubsystem extends SubsystemBase {
//  private static final ShooterSubsystem INSTANCE = new ShooterSubsystem();
//
//  public static ShooterSubsystem getInstance() {
//    return INSTANCE;
//  }
//
//  public boolean intake = true;
//  private SparkMax intakeMotor = new SparkMax(14, SparkLowLevel.MotorType.kBrushless);
//
//  public sparkmaxSubsystem() {
//    // Example configuration — real code uses SparkBaseConfig
//    SparkMaxConfig config = new SparkMaxConfig();
//    config.idleMode(SparkBaseConfig.IdleMode.kCoast);
//    config.smartCurrentLimit(40);
//    intakeMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
//  }
//
//  @Override
//  public void periodic() {
//    if (intake) {
//      intakeMotor.set(0.45);
//    } else {
//      intakeMotor.set(0.0);
//    }
//  }
// }
