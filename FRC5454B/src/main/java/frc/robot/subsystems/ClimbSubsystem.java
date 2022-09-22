package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxAlternateEncoder;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax.IdleMode;
import frc.robot.Constants;

public class ClimbSubsystem extends SubsystemBase {
    private CANSparkMax m_ClimbMotor1;
    private CANSparkMax m_ClimbMotor2;

    public ClimbSubsystem(int ClimbMotor1Port, int ClimbMotor2Port){
        m_ClimbMotor1 = new CANSparkMax(ClimbMotor1Port, MotorType.kBrushed);   
        m_ClimbMotor2 = new CANSparkMax(ClimbMotor2Port, MotorType.kBrushed);      
        
      }
    
    @Override
    public void periodic() {
      // This method will be called once per scheduler run
    }

    public void runClimb(double power){
      System.out.println("Setting Power on Climb - " + power);
      m_ClimbMotor1.set(power);
      m_ClimbMotor2.set(power);
    }

    public void stopClimb(){
      m_ClimbMotor1.set(0);
      m_ClimbMotor2.set(0);
    }

    @Override
    public void simulationPeriodic() {
      // This method will be called once per scheduler run during simulation
    }
}
