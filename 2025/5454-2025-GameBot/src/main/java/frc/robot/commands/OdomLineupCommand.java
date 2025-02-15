package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.LimeLightValues.LimelightLineUpOffsets;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.utilities.Limelight;
import frc.robot.utilities.AutoPlanner;

public class OdomLineupCommand extends Command{
    private Limelight m_limeLight;
    private CommandSwerveDrivetrain m_swerve;

    private Pose2d m_target;

    public OdomLineupCommand(Limelight limeLight,CommandSwerveDrivetrain swerve,Pose2d target){
        m_limeLight=limeLight;
        m_swerve=swerve;
        m_target=target;
    }

    @Override
    public void initialize(){
        AutoPlanner autoPlan=new AutoPlanner();
        try{
                            
        Command newCommand=m_swerve.createPathCommand(autoPlan.CreateOdomLineUpPath(m_swerve.getPose2d(),m_target));
        CommandScheduler.getInstance().schedule(newCommand);
        }catch(Exception e){}
    }

    @Override
    public void end(boolean interrupted){}

    @Override
    public boolean isFinished(){
        return true;
    }
}
