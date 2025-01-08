package frc.robot.subsystems;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TestSubsystem2 extends SubsystemBase{
    private TalonSRX m_motor;
  
    public TestSubsystem2(int port){
        m_motor= new TalonSRX(port);
        m_motor.configFactoryDefault();
        
    
        }

    public void run(double speed){
        m_motor.set(TalonSRXControlMode.PercentOutput, speed);
       
    }
    public void stop(){
        m_motor.set(TalonSRXControlMode.PercentOutput,0);
        
    }
    
    
}
