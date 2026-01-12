package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class ResetGyroCommand extends Command{
    private CommandSwerveDrivetrain m_swerve;
    
    public ResetGyroCommand(CommandSwerveDrivetrain swerve){
        m_swerve=swerve;
    }

    @Override
    public void initialize() {
    }

    @Override
    public void end(boolean interrupted){
    }

    @Override
    public boolean isFinished(){
        return true;
    }

}
