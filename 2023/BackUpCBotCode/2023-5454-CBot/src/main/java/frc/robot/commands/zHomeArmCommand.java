package frc.robot.commands;

import frc.robot.subsystems.ArmSubsystem;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class zHomeArmCommand extends CommandBase {
    private ArmSubsystem m_ArmSubsystem;
    DigitalInput m_limitswitch;
    private double m_rotateSpeed;
    ArmSubsystem m_homePos;
    ArmSubsystem m_leftArmMotor;
    
    public zHomeArmCommand(ArmSubsystem subsystem, DigitalInput limitswitch, double rotateSpeed){
        m_limitswitch=limitswitch;
        m_ArmSubsystem=subsystem;
        m_rotateSpeed=rotateSpeed;
        }

    public void setSpeed(double rotateSpeed){
        rotateSpeed=0.1;
    }
    
    public void initialize(){}

    public void execute(){}

    public boolean isFinished(){
        if(m_limitswitch.get() == false){
            m_ArmSubsystem.rotateArm(m_rotateSpeed);
            return false;
        }else{
            m_ArmSubsystem.stopRotate();
            m_ArmSubsystem.setHomePos();
            return true;
        }
        
    }

    }