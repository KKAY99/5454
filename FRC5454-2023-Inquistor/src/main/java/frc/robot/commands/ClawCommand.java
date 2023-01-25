package frc.robot.commands;
import frc.robot.subsystems.ClawSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ClawCommand extends CommandBase {
    private ClawSubsystem m_ClawSubystem;
    private double m_speed;
    public ClawCommand(ClawSubsystem ClawSubystem,double speed){
        m_ClawSubystem=ClawSubystem;
        m_speed=speed;
    }

    @Override
    public void initialize() {
    }  

    @Override
    public void execute() {
    
    }
  
    @Override
    public void end(boolean interrupted) {
    
    }
  
    @Override
    public boolean isFinished() {
     return false;
    }

}
