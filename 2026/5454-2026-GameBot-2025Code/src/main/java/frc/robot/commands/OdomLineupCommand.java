package frc.robot.commands;

import java.util.function.Supplier;

import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.LimeLightValues;
import frc.robot.Constants.LineupConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.utilities.Limelight;
import frc.robot.utilities.AutoPlanner;

public class OdomLineupCommand extends Command{
    private Limelight m_limeLight;
    private CommandSwerveDrivetrain m_swerve;

    private Pose2d m_target;
    private Pose2d m_targetLineup;

    private Supplier<Boolean> m_isRightLineup;

    public OdomLineupCommand(Limelight limeLight,CommandSwerveDrivetrain swerve,Supplier<Boolean> isRightLineup){
        m_limeLight=limeLight;
        m_swerve=swerve;
        m_isRightLineup=isRightLineup;
    }

    @Override
    public void initialize(){
        AutoPlanner autoPlan=new AutoPlanner();

        if(m_limeLight.isAnyTargetAvailable()){
            try{
                Pose2d startingPose2d=null;

                if(DriverStation.getAlliance().get()==Alliance.Blue){
                    m_limeLight.setCodeIDFilter(17,18,19,20,21,22);
                    int currentFiducial=m_limeLight.getFirstVisibleFiducialID();
                    startingPose2d=m_swerve.getPose2d();
                    if(m_isRightLineup.get()){
                        m_target=LineupConstants.fiducialBlueRightPoses[currentFiducial-17]; 
                        m_targetLineup=LineupConstants.fiducialBlueRightLineupPoses[currentFiducial-17]; 
                    }else{
                        m_target=LineupConstants.fiducialBlueLeftPoses[currentFiducial-17]; 
                        m_targetLineup=LineupConstants.fiducialBlueLeftLineupPoses[currentFiducial-17];
                    }
                }else{
                    m_limeLight.setCodeIDFilter(6,7,8,9,10,11);
                    int currentFiducial=m_limeLight.getFirstVisibleFiducialID();
                    startingPose2d=FlippingUtil.flipFieldPose(m_swerve.getPose2d());
                    if(m_isRightLineup.get()){
                        m_target=LineupConstants.fiducialBlueRightPoses[currentFiducial-6]; 
                        m_targetLineup=LineupConstants.fiducialBlueRightLineupPoses[currentFiducial-6];
                    }else{
                        m_target=LineupConstants.fiducialBlueLeftPoses[currentFiducial-6]; 
                        m_targetLineup=LineupConstants.fiducialBlueLeftLineupPoses[currentFiducial-6];
                    }
                } 
                Command newCommand=m_swerve.createPathCommand(autoPlan.CreateOdomLineUpPath(startingPose2d,m_target,m_targetLineup));
                CommandScheduler.getInstance().schedule(newCommand);
            }catch(Exception e){}
        }else{
        }
    }

    @Override
    public void end(boolean interrupted){
        m_swerve.drive(0,0,0);
    }

    @Override
    public boolean isFinished(){
        boolean returnValue=false;
        Pose2d currentPose=m_swerve.getPose2d();

        if(m_target!=null){
            if(currentPose.getX()-LineupConstants.lineUpDeadband<m_target.getX()&&currentPose.getX()+LineupConstants.lineUpDeadband>m_target.getX()&&
            currentPose.getY()-LineupConstants.lineUpDeadband<m_target.getY()&&currentPose.getY()+LineupConstants.lineUpDeadband>m_target.getY()){
                returnValue=true;
            }
        }else{
            returnValue=true;
        }

        return returnValue;
    }
}
