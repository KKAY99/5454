package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.utilities.Limelight;
import frc.robot.utilities.AutoPlanner;

public class OdomLineupCommand extends Command{
    private Limelight m_limeLight;
    private CommandSwerveDrivetrain m_swerve;

    public OdomLineupCommand(Limelight limeLight,CommandSwerveDrivetrain swerve){
        m_limeLight=limeLight;
        m_swerve=swerve;
    }

    @Override
    public void initialize(){
        Command newCommand=m_swerve.createPathCommand(new AutoPlanner().CreateOdomLineUpPath(m_limeLight.findGlobalPoseFromTargetPoseRobotSpace(
                            m_swerve.getPigeon2().getAngularVelocityZWorld().getValueAsDouble())));

        CommandScheduler.getInstance().schedule(newCommand);
    }

    @Override
    public boolean isFinished(){
        return true;
    }
}
