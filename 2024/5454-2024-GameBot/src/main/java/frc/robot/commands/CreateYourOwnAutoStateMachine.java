package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.IntakeConstants;
import java.util.List;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.utilities.AutoPose2D;
import frc.robot.utilities.Limelight;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.Constants;

public class CreateYourOwnAutoStateMachine extends Command{
    private IntakeSubsystem m_intake;
    private Limelight m_limelight;
    private ShooterSubsystem m_shooter;
    private TurretSubsystem m_turret;
    private Swerve m_swerve;

    private CommandScheduler m_commandScheduler=CommandScheduler.getInstance();

    private enum PoseStates{SETNEWPOSES,WAIT}; 
    private enum WaypointStates{START,INTAKEPOS,INTAKEPOSWAIT,STARTINTAKE,NOTEPOS,NOTEPOSWAIT,STOPINTAKE,SHOOTPOS,SHOOTPOSWAIT,SHOOT,SHOOTWAIT,END};

    private PoseStates m_poseState;
    private WaypointStates m_waypointState;

    private AutoPose2D[] m_poses;
    private Pose2d m_intakePos;
    private Pose2d m_notePos;
    private Pose2d m_shotPos;

    private int m_currentPose=0;
    private int m_endIndex;

    private Command m_intakePathCommand;
    private Command m_notePathCommand;
    private Command m_shootPathCommand;
    private Command m_shoot=new SmartShooter(m_shooter,m_turret,m_swerve, m_limelight, m_intake,false,false,true);
    private Command m_toggleIntake=new IntakeToggleCommand(m_intake,Constants.IntakeConstants.intakeSpeed,true);

    public CreateYourOwnAutoStateMachine(Swerve swerve,ShooterSubsystem shooter,TurretSubsystem turret,IntakeSubsystem intake,Limelight limelight,AutoPose2D... poses){
        m_intake=intake;
        m_swerve=swerve;
        m_shooter=shooter;
        m_turret=turret;
        m_limelight=limelight;
        m_endIndex=0;

        int arrayPos=0;

        for(AutoPose2D pose:poses){
            if(pose!=null){
                m_poses[arrayPos]=pose;
                arrayPos++;
            }
        }

        if(m_poses.length!=0){
            m_endIndex=m_poses.length+1;
        }
    }

    private void IndexPoseState(){
        m_currentPose++;
        m_poseState=PoseStates.SETNEWPOSES;
        m_waypointState=WaypointStates.START;
    }

    private PathPlannerPath CreateAutoPath(Pose2d startPose,Pose2d desiredPose){
    List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(
      startPose,
      desiredPose
    );

    PathPlannerPath newPath = new PathPlannerPath(
          bezierPoints,
          new PathConstraints(3.0, 3.0, 2 * Math.PI, 4 * Math.PI), // The constraints for this path. If using a differential drivetrain, the angular constraints have no effect.
          new GoalEndState(0.0, Rotation2d.fromDegrees(0)) // Goal end state. You can set a holonomic rotation here. If using a differential drivetrain, the rotation will have no effect.
    );

    return newPath;
  }

    @Override
    public void initialize(){
        m_poseState=PoseStates.SETNEWPOSES;
        m_waypointState=WaypointStates.START;

        m_currentPose=0;
    }

    @Override
    public boolean isFinished(){
        boolean shouldEndAuto=false;

        switch(m_poseState){
            case SETNEWPOSES:
                if(m_endIndex!=m_currentPose){
                    m_intakePos=m_poses[m_currentPose].getIntakePos();
                    m_notePos=m_poses[m_currentPose].getPathPos();
                    m_shotPos=m_poses[m_currentPose].getShotPos();

                    m_poseState=PoseStates.WAIT;
                }else{
                    shouldEndAuto=true;
                }
            break;
            case WAIT:
            break;
        }

        switch(m_waypointState){
            case START:
                if(!shouldEndAuto){
                    m_waypointState=WaypointStates.INTAKEPOS;
                }
            break;
            case INTAKEPOS:
                m_intakePathCommand=m_swerve.createPathCommand(CreateAutoPath(m_swerve.getPose(),m_intakePos));

                m_commandScheduler.schedule(m_intakePathCommand);
                m_waypointState=WaypointStates.INTAKEPOSWAIT;
            break;
            case INTAKEPOSWAIT:
                if(!m_commandScheduler.isScheduled(m_intakePathCommand)){
                    m_waypointState=WaypointStates.STARTINTAKE;
                }
            break;
            case STARTINTAKE:
                m_commandScheduler.schedule(m_toggleIntake);
                m_waypointState=WaypointStates.NOTEPOS;
            break;
            case NOTEPOS:
                m_notePathCommand=m_swerve.createPathCommand(CreateAutoPath(m_swerve.getPose(),m_notePos));

                m_commandScheduler.schedule(m_notePathCommand);
                m_waypointState=WaypointStates.NOTEPOSWAIT;
            break;
            case NOTEPOSWAIT:
                if(!m_commandScheduler.isScheduled(m_notePathCommand)){
                    m_waypointState=WaypointStates.STOPINTAKE;
                }
            break;
            case STOPINTAKE:
                m_commandScheduler.schedule(m_toggleIntake);
                m_waypointState=WaypointStates.SHOOTPOS;
            break;
            case SHOOTPOS:
                m_shootPathCommand=m_swerve.createPathCommand(CreateAutoPath(m_swerve.getPose(),m_shotPos));

                m_commandScheduler.schedule(m_shootPathCommand);
                m_waypointState=WaypointStates.SHOOTPOSWAIT;
            break;
            case SHOOTPOSWAIT:
                if(!m_commandScheduler.isScheduled(m_shootPathCommand)){
                    m_waypointState=WaypointStates.SHOOT;
                }
            break;
            case SHOOT:
                m_commandScheduler.schedule(m_shoot);
                m_waypointState=WaypointStates.SHOOTWAIT;
            break;
            case SHOOTWAIT:
                if(!m_commandScheduler.isScheduled(m_shoot)){
                    m_waypointState=WaypointStates.END;
                }
            break;
            case END:
            //check to see if at last Pos
            IndexPoseState();
        }

        System.out.println("Current Pose State: "+m_poseState);
        System.out.println("Current Waypoint State: "+m_waypointState);
        return shouldEndAuto;
    }

}
