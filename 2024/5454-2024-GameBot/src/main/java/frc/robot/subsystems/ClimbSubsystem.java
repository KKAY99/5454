package frc.robot.subsystems;
import com.fasterxml.jackson.annotation.JsonTypeInfo.Id;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimbSubsystem extends SubsystemBase{

    private CANSparkMax m_climbOne;
    public ClimbSubsystem(int motorOne){
        m_climbOne= new CANSparkMax(motorOne,MotorType.kBrushless);
        m_climbOne.setIdleMode(IdleMode.kBrake);
       
        }

    public void runClimb(double speed){
        m_climbOne.set(speed);
        
    }
    public void stopClimb(){
        m_climbOne.set(0);
        
    }
            
    public void setBrakeOn(){
      m_climbOne.setIdleMode(IdleMode.kBrake);
     
    } 
    public void setCoastOn(){
         m_climbOne.setIdleMode(IdleMode.kCoast);
    }
    
}
