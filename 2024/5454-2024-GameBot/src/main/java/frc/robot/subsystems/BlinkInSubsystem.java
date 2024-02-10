package frc.robot.subsystems;
import org.littletonrobotics.junction.Logger;

import com.fasterxml.jackson.annotation.JsonTypeInfo.Id;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class BlinkInSubsystem extends SubsystemBase{
    private Spark m_blinkIn;

    private double m_pattern;

    public BlinkInSubsystem(int PWMPort){
        m_blinkIn= new Spark(PWMPort);
       
    }

    public void runled(double pattern){
        m_pattern=pattern;
        m_blinkIn.set(pattern);  
    }

    public void stopled(){
        m_pattern=0;
        m_blinkIn.set(0);  
    }
            
    @Override
    public void periodic(){
        Logger.recordOutput("LED-BlinkIn/Pattern",m_pattern);
    }
}
