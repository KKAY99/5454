package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.IntakeConstants;
import java.util.ArrayList;
import java.util.List;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
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
    private enum WaypointStates{START,INITALSHOOT,INITALSHOOTWAIT,TURRETSTRAIGHT,TURRETSTRAIGHTWAIT,LONGNOTESTARTPOS,LONGNOTESTARTPOSWAIT,INTAKEPOS,INTAKEPOSWAIT,
                                SHOOTERINTAKEANGLE,SHOOTERINTAKEANGLEWAIT,STARTINTAKE,NOTEPOS,NOTEPOSWAIT,STOPINTAKE,SHOOTPOS,SHOOTPOSWAIT,SHOOT,SHOOTWAIT,END};

    private PoseStates m_poseState;
    private WaypointStates m_waypointState;

    private List<AutoPose2D> m_poses=new ArrayList<>();
    private Pose2d m_intakePos;
    private Pose2d m_notePos;
    private Pose2d m_shotPos;
    private Pose2d m_arrayIntakePos;
    private Pose2d m_arrayNotePos;
    private Pose2d m_arrayShotPos;

    private Command m_intakePathCommand;
    private Command m_notePathCommand;
    private Command m_shootPathCommand;
    private Command m_longNoteStartPosCommand;
    private Command m_shoot;
    private Command m_shooterSetRef;
    private Command m_intakeCommand;
    private Command m_turretStraight;

    private int m_currentPose=0;
    private int m_endIndex;

    public CreateYourOwnAutoStateMachine(Swerve swerve,ShooterSubsystem shooter,TurretSubsystem turret,IntakeSubsystem intake,Limelight limelight,AutoPose2D... poses){
        m_intake=intake;
        m_swerve=swerve;
        m_shooter=shooter;
        m_turret=turret;
        m_limelight=limelight;
        m_endIndex=0;

        for(AutoPose2D pose:poses){
            if(pose!=null){
                m_poses.add(pose);
            }
        }

        if(m_poses.size()!=0){
            m_endIndex=m_poses.size();
        }

        CreateCommands();
    }

    public void CreateCommands(){
        m_shoot=new SmartShooter(m_shooter, m_turret, m_swerve, m_limelight, m_intake,false,false,true,false);
        m_intakeCommand=new IntakeToggleCommand(m_intake,Constants.IntakeConstants.autoIntakeSpeed,true);
        m_shooterSetRef=new ShootRotateSetReferenceCommand(m_shooter,0);
        m_turretStraight=new TurretPosCommand(m_turret,Constants.TurretConstants.turretStraightPos,Constants.TurretConstants.turretMoveTimeOut,Constants.TurretConstants.deadband);
    }

    /**
     * Indexs a list of poses to the next element
    */
    private void IndexPoseState(){
        m_currentPose++;
        m_poseState=PoseStates.SETNEWPOSES;
        m_waypointState=WaypointStates.START;
    }

    /**
     * Used for blue side autos due to pathplanner issues
     * @param pose 
     * @return Inverted Pose2d
    */
    private Pose2d InvertPose(Pose2d pose){
        Pose2d newPose2d=new Pose2d(pose.getX()*-1,
                                    pose.getY()*-1,
                                    pose.getRotation().unaryMinus());

        return newPose2d;
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
        Alliance currentAlliance=DriverStation.getAlliance().get();

        switch(m_poseState){
            case SETNEWPOSES:
                if(m_endIndex!=m_currentPose){
                    if(currentAlliance==Alliance.Blue){
                        m_arrayIntakePos=InvertPose(m_poses.get(m_currentPose).getIntakePos());
                        m_arrayNotePos=InvertPose(m_poses.get(m_currentPose).getNotePos());
                        m_arrayShotPos=InvertPose(m_poses.get(m_currentPose).getShotPos());
                    }else{
                        m_arrayIntakePos=m_poses.get(m_currentPose).getIntakePos();
                        m_arrayNotePos=m_poses.get(m_currentPose).getNotePos();
                        m_arrayShotPos=m_poses.get(m_currentPose).getShotPos();
                    }

                    m_intakePos=m_arrayIntakePos;
                    m_notePos=m_arrayNotePos;
                    m_shotPos=m_arrayShotPos;

                    m_poseState=PoseStates.WAIT;
                }else{
                    shouldEndAuto=true;
                }
            break;
            case WAIT:
                //Does nothing until m_poses indexs
            break;
        }

        switch(m_waypointState){
            case START:
                if(!shouldEndAuto&&m_currentPose!=0){
                    if(!m_poses.get(m_currentPose).IsLongNote()){
                        m_waypointState=WaypointStates.TURRETSTRAIGHT;
                    }else{
                        m_waypointState=WaypointStates.LONGNOTESTARTPOS;
                    }
                }

                if(m_currentPose==0){
                    m_waypointState=WaypointStates.INITALSHOOT;
                }
            break;
            case INITALSHOOT:
                m_commandScheduler.schedule(m_shoot);
                m_waypointState=WaypointStates.INITALSHOOTWAIT;
            break;
            case INITALSHOOTWAIT:
                if(!m_commandScheduler.isScheduled(m_shoot)){
                    if(!m_poses.get(m_currentPose).IsLongNote()){
                        m_waypointState=WaypointStates.TURRETSTRAIGHT;
                    }else{
                        m_waypointState=WaypointStates.LONGNOTESTARTPOS;
                    };
                }
            break;
            case LONGNOTESTARTPOS:
                if(currentAlliance==Alliance.Blue){
                    if(Math.abs(m_swerve.getPose().getY())>AutoConstants.yCoordMidpoint){
                        m_longNoteStartPosCommand=m_swerve.createPathCommand(CreateAutoPath(m_swerve.getPose(),AutoConstants.locationBlueLongAmpWing));
                    }else{
                        m_longNoteStartPosCommand=m_swerve.createPathCommand(CreateAutoPath(m_swerve.getPose(),AutoConstants.locationBlueLongSourceWing));
                    }
                }else{
                    if(m_swerve.getPose().getY()>AutoConstants.yCoordMidpoint){
                        m_longNoteStartPosCommand=m_swerve.createPathCommand(CreateAutoPath(m_swerve.getPose(),AutoConstants.locationRedLongAmpWing));
                    }else{
                        m_longNoteStartPosCommand=m_swerve.createPathCommand(CreateAutoPath(m_swerve.getPose(),AutoConstants.locationRedLongSourceWing));
                    }
                }

                m_waypointState=WaypointStates.LONGNOTESTARTPOSWAIT;
            break;
            case LONGNOTESTARTPOSWAIT:
                if(!m_commandScheduler.isScheduled(m_longNoteStartPosCommand)){
                    m_waypointState=WaypointStates.TURRETSTRAIGHT;
                }
            break;
            case TURRETSTRAIGHT:
                m_commandScheduler.schedule(m_turretStraight);
                m_waypointState=WaypointStates.TURRETSTRAIGHTWAIT;     
            break;
            case TURRETSTRAIGHTWAIT:
                if(!m_commandScheduler.isScheduled(m_turretStraight)){
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
                    m_waypointState=WaypointStates.SHOOTERINTAKEANGLE;
                }
            break;
            case SHOOTERINTAKEANGLE:
                m_commandScheduler.schedule(m_shooterSetRef);
                m_waypointState=WaypointStates.SHOOTERINTAKEANGLEWAIT;
            break;
            case SHOOTERINTAKEANGLEWAIT:
                if(!m_commandScheduler.isScheduled(m_shooterSetRef)){
                    m_waypointState=WaypointStates.STARTINTAKE;
                }
            break;
            case STARTINTAKE:
                m_commandScheduler.schedule(m_intakeCommand);
                m_waypointState=WaypointStates.NOTEPOS;
            break;
            case NOTEPOS:
                m_notePathCommand=m_swerve.createPathCommand(CreateAutoPath(m_swerve.getPose(),m_notePos));

                m_commandScheduler.schedule(m_notePathCommand);
                m_waypointState=WaypointStates.NOTEPOSWAIT;
            break;
            case NOTEPOSWAIT:
                if(!m_commandScheduler.isScheduled(m_notePathCommand)){
                    m_waypointState=WaypointStates.SHOOTPOS;
                }
            break;
            case STOPINTAKE:
                m_commandScheduler.schedule(m_intakeCommand);
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
            IndexPoseState();
        }

        System.out.println("Current Waypoint State: "+m_waypointState);
        return shouldEndAuto;
    }

}
