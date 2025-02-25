package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.LineupConstants;
import frc.robot.Constants.LimeLightValues.LimelightLineUpOffsets;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.utilities.Limelight;
import frc.robot.utilities.AutoPlanner;

public class OdomLineupCommand extends Command{
    private Limelight m_limeLight;
    private CommandSwerveDrivetrain m_swerve;

    private Pose2d m_target;

    private boolean m_isRightLineup;

    public OdomLineupCommand(Limelight limeLight,CommandSwerveDrivetrain swerve,boolean isRightLineup){
        m_limeLight=limeLight;
        m_swerve=swerve;
        m_isRightLineup=isRightLineup;
    }

    @Override
    public void initialize(){
        AutoPlanner autoPlan=new AutoPlanner();

        if(m_limeLight.isAnyTargetAvailable()){
            System.out.println("CURRENT TARGET ID"+m_limeLight.getAllVisibleFiducialIDs()[0]);
            try{
                if(DriverStation.getAlliance().get()==Alliance.Blue){
                    m_limeLight.setCodeIDFilter(17,18,19,20,21,22);
                    int currentFiducial=m_limeLight.getAllVisibleFiducialIDs()[0];
                    if(m_isRightLineup){
                        m_target=LineupConstants.fiducialBlueRightPoses[currentFiducial-17]; 
                    }else{
                        m_target=LineupConstants.fiducialBlueLeftPoses[currentFiducial-17]; 
                    }
                }else{
                    m_limeLight.setCodeIDFilter(6,7,8,9,10,11);
                    int currentFiducial=m_limeLight.getAllVisibleFiducialIDs()[0];
                    if(m_isRightLineup){
                        m_target=LineupConstants.fiducialBlueRightPoses[currentFiducial-6]; 
                        Translation2d flippedPoint=FlippingUtil.flipFieldPosition(m_target.getTranslation());
                        m_target=new Pose2d(flippedPoint.getX(),flippedPoint.getY(),m_target.getRotation());
                    }else{
                        m_target=LineupConstants.fiducialBlueLeftPoses[currentFiducial-6]; 
                        Translation2d flippedPoint=FlippingUtil.flipFieldPosition(m_target.getTranslation());
                        m_target=new Pose2d(flippedPoint.getX(),flippedPoint.getY(),m_target.getRotation());
                    }
                } 
                System.out.println("Moving to "+ m_target.toString());
                Command newCommand=m_swerve.createPathCommand(autoPlan.CreateOdomLineUpPath(m_swerve.getPose2d(),m_target));
                CommandScheduler.getInstance().schedule(newCommand);
            }catch(Exception e){}
        }else{
            System.out.println("NO TARGET");
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
