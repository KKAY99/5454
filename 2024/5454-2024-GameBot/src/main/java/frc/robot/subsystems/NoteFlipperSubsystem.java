package frc.robot.subsystems;
import java.io.Console;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import frc.robot.utilities.ObsidianCANSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class NoteFlipperSubsystem extends SubsystemBase{
    private ObsidianCANSparkMax m_motor;

    public NoteFlipperSubsystem(int canid){
        final boolean kisBrakeMode=false;
        m_motor=new ObsidianCANSparkMax(canid,MotorType.kBrushless,kisBrakeMode,Constants.k30Amp);
     
    }

    public void run(double power){
        m_motor.set(power);
    }

    public void stop(){
        m_motor.stopMotor();
    }
    
}
