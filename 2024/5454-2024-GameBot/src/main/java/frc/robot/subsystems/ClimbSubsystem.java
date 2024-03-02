package frc.robot.subsystems;
import org.littletonrobotics.junction.Logger;

import com.fasterxml.jackson.annotation.JsonTypeInfo.Id;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimbSubsystem extends SubsystemBase{
    private CANSparkMax m_climbOne;

    private double m_speed;

    private double m_isExtended;

    public ClimbSubsystem(int motorOne){
        m_climbOne= new CANSparkMax(motorOne,MotorType.kBrushless);
        m_climbOne.setSmartCurrentLimit(Constants.k30Amp);
        m_climbOne.setIdleMode(IdleMode.kBrake);
        m_climbOne.burnFlash();
       
    }

    public void runClimb(double speed){
        m_speed=speed;
        m_climbOne.set(speed);  
    }

    public void stopClimb(){
        m_speed=0;
        m_climbOne.set(0);  
    }
            
    public void setBrakeOn(){
      m_climbOne.setIdleMode(IdleMode.kBrake);
     
    } 

    public void setCoastOn(){
         m_climbOne.setIdleMode(IdleMode.kCoast);
    }

    @Override
    public void periodic(){
        Logger.recordOutput("Climb/ClimbSpeed",m_speed);
        Logger.recordOutput("Climb/IsExtended",m_isExtended);
    }
}
