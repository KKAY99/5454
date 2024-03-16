package frc.robot.subsystems;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Lasercan;

public class TestSubsystem1 extends SubsystemBase{
    private CANSparkMax m_motor;
  
    public TestSubsystem1(int port){
        m_motor= new CANSparkMax(port,CANSparkLowLevel.MotorType.kBrushed);
    
        }

    public void run(double speed){
        m_motor.set(speed);
       
    }
    public void stop(){
        m_motor.set(0);
        
    }
    
    
}
