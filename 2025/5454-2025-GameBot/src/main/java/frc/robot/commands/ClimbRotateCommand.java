package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimbSubsystem;

public class ClimbRotateCommand extends Command{
    private ClimbSubsystem m_climb;

    private double m_speed;
    
    public ClimbRotateCommand(ClimbSubsystem climb,double speed){
        m_climb=climb;
        m_speed=speed;
    }

    @Override
    public void initialize(){
        m_climb.resetPID();
    }

    @Override
    public void end(boolean interrupted){
        m_climb.stop();
    }

    @Override
    public boolean isFinished(){
        m_climb.runWithLimits(m_speed);
        return false;
    }
}
