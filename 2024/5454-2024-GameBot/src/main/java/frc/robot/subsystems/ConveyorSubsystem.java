package frc.robot.subsystems;

import com.fasterxml.jackson.annotation.JsonTypeInfo.Id;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ConveyorSubsystem extends SubsystemBase{

    private CANSparkMax m_conveyOne;
    public ConveyorSubsystem(int motorOne){
        m_conveyOne= new CANSparkMax(motorOne,MotorType.kBrushless);
        m_conveyOne.setIdleMode(IdleMode.kBrake);
       
        }

    public void runConveyor(double speed){
        m_conveyOne.set(speed);
        
    }
    public void stopClimb(){
        m_conveyOne.set(0);
        
    }
            
    public void setBrakeOn(){
      m_conveyOne.setIdleMode(IdleMode.kBrake);
     
    } 
    public void setCoastOn(){
         m_conveyOne.setIdleMode(IdleMode.kCoast);
    }
    
}

