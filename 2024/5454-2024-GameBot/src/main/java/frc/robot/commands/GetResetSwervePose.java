package frc.robot.commands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.utilities.Limelight;
import frc.robot.subsystems.Swerve;

public class GetResetSwervePose extends Command{
    private Swerve m_swerve;
    private Limelight m_limelight;

    public GetResetSwervePose(Swerve swerve,Limelight limeLight){
        m_swerve=swerve;
        m_limelight=limeLight;
    }

    @Override
    public void execute(){
        if(m_limelight.isTargetAvailible()){
            Pose2d newPose=m_limelight.GetPoseViaApriltag();

            m_swerve.resetOdometry(newPose);
        }
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}

