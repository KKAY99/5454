package frc.robot.subsystems;
import java.io.Console;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class NoteFlipperSubsystem extends SubsystemBase{
    private CANSparkMax m_motor;

    public NoteFlipperSubsystem(int canid){
        m_motor=new CANSparkMax(canid,MotorType.kBrushless);
        m_motor.setPeriodicFramePeriod(PeriodicFrame.kStatus3,1000);
        m_motor.setPeriodicFramePeriod(PeriodicFrame.kStatus4,1000);
        m_motor.setPeriodicFramePeriod(PeriodicFrame.kStatus5,1000);
        m_motor.setPeriodicFramePeriod(PeriodicFrame.kStatus6,1000);   
        m_motor.setSmartCurrentLimit(Constants.k30Amp);
        m_motor.burnFlash();
    }

    public void run(double power){
        m_motor.set(power);
    }

    public void stop(){
        m_motor.stopMotor();
    }
    
}
