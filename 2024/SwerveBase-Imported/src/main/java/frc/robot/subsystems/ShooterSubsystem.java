package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class ShooterSubsystem extends SubsystemBase{
    private CANSparkMax m_motor1;
    private CANSparkMax m_motor2;

    public ShooterSubsystem(int motorPort1, int motorPort2){
        m_motor1=new CANSparkMax(motorPort1,MotorType.kBrushed);
        m_motor2=new CANSparkMax(motorPort2,MotorType.kBrushed);
    }
    
    public void run(double power){
        m_motor1.set(power);
        m_motor2.set(power);
    }
 
    public void stop(){
        m_motor1.stopMotor();
        m_motor2.stopMotor();
    }
}

