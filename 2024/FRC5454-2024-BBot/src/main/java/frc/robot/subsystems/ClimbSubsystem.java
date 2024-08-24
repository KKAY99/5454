package frc.robot.subsystems;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.classes.ObsidianCANSparkMax;

public class ClimbSubsystem extends SubsystemBase {
  private ObsidianCANSparkMax m_motor1;


  public ClimbSubsystem(int motor1ID) {
    m_motor1 = new ObsidianCANSparkMax (motor1ID, MotorType.kBrushless, true);
  
  }
  public void run(double power) {
    m_motor1.set(power);

  }
  public void stop() {
    m_motor1.set(0);
   
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run

  }
}
