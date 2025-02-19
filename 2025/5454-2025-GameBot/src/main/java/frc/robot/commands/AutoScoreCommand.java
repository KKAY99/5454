package frc.robot.commands;
import java.security.spec.ECPublicKeySpec;
import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.DunkinDonutSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.utilities.AutoPlanner;
import frc.robot.utilities.Limelight;
import frc.robot.Constants.DunkinDonutConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.ElevatorConstants.ElevatorScoreLevel;
import frc.robot.Constants.LineupConstants;
import java.util.function.Supplier;

public class AutoScoreCommand extends Command{
    private CommandSwerveDrivetrain m_swerve;
    private ElevatorSubsystem m_elevator;
    private DunkinDonutSubsystem m_dunkin;
    private Limelight m_limeLight;

    private Supplier<ElevatorScoreLevel> m_scoreLevel;

    private Pose2d m_odomTarget;

    private double m_elevatorIPos;
    private double m_elevatorFPos;
    private double m_algaePos;
    private double m_startTime;

    private boolean m_isRightLineup;
    private boolean m_isManual=false;
    
    private enum States{
        CHECKFORTARGET,PRESCOREELEV,LINEUP,WAITFORLINEUP,ALGAE,ELEVATOR,WAITFORELEVATOR,CORAL,RETRACT,END
    }

    private States m_currentState=States.CHECKFORTARGET;

    public AutoScoreCommand(ElevatorSubsystem elevator,DunkinDonutSubsystem dunkin,Limelight limeLight,Supplier<ElevatorScoreLevel> scorelevel,boolean isRightLineup){
        m_elevator=elevator;
        m_dunkin=dunkin;
        m_limeLight=limeLight;

        m_scoreLevel=scorelevel;
        m_isRightLineup=isRightLineup;
        m_isManual=true;

        addRequirements(m_elevator,m_dunkin);
    }

    public AutoScoreCommand(CommandSwerveDrivetrain swerve,ElevatorSubsystem elevator,DunkinDonutSubsystem dunkin,Limelight limeLight,Supplier<ElevatorScoreLevel> scorelevel,
                            boolean isRightLineup){
        m_swerve=swerve;
        m_elevator=elevator;
        m_dunkin=dunkin;
        m_limeLight=limeLight;

        m_scoreLevel=scorelevel;
        m_isRightLineup=isRightLineup;

        addRequirements(m_swerve,m_elevator,m_dunkin);
    }

    @Override
    public void initialize(){
        m_currentState=States.CHECKFORTARGET;
        switch(m_scoreLevel.get()){
            case L1:
            m_elevatorFPos=ElevatorConstants.l1Pos;
            m_elevatorIPos=ElevatorConstants.l1Pos;
            break;
            case L2:
            m_elevatorFPos=ElevatorConstants.l2Pos;
            m_elevatorIPos=ElevatorConstants.l2Pos;
            break;
            case L3:
            m_elevatorFPos=ElevatorConstants.l3Pos;
            m_elevatorIPos=ElevatorConstants.l3Pos;
            break;
            case L4:
            m_elevatorFPos=ElevatorConstants.l4Pos;
            m_elevatorIPos=ElevatorConstants.l3Pos;
            break; 
        }
    }

    @Override
    public void end(boolean interrupted) {
        m_dunkin.stopAlgeaMotor();
        m_dunkin.stopCoralMotor();
        m_elevator.reset_referance();
        m_elevator.motor_stop();
        m_dunkin.resetShouldRunPID();
    }

    @Override
    public boolean isFinished(){
        boolean returnValue=false;

        switch(m_currentState){
        case CHECKFORTARGET:     
            if(m_isManual){
                m_currentState=States.ALGAE;
            }else{
                if(m_limeLight.isAnyTargetAvailable()){
                    m_currentState=States.PRESCOREELEV;
                }else{
                    m_currentState=States.END;
                }
            }
        break;
        case PRESCOREELEV:
            m_elevator.set_referance(m_elevatorIPos);

            m_currentState=States.LINEUP;
        break;
        case LINEUP:
            AutoPlanner autoPlan=new AutoPlanner();
            if(m_limeLight.isAnyTargetAvailable()){
                try{
                    if(DriverStation.getAlliance().get()==Alliance.Blue){
                        m_limeLight.setCodeIDFilter(17,18,19,20,21,22);
                        int currentFiducial=m_limeLight.getAllVisibleFiducialIDs()[0];
                        if(m_isRightLineup){
                            m_odomTarget=LineupConstants.fiducialBlueRightPoses[currentFiducial-17]; 
                        }else{
                            m_odomTarget=LineupConstants.fiducialBlueLeftPoses[currentFiducial-17]; 
                        }
                    }else{
                        m_limeLight.setCodeIDFilter(6,7,8,9,10,11);
                        int currentFiducial=m_limeLight.getAllVisibleFiducialIDs()[0];
                        if(m_isRightLineup){
                            m_odomTarget=LineupConstants.fiducialBlueRightPoses[currentFiducial-6]; 
                            Translation2d flippedPoint=FlippingUtil.flipFieldPosition(m_odomTarget.getTranslation());
                            m_odomTarget=new Pose2d(flippedPoint.getX(),flippedPoint.getY(),m_odomTarget.getRotation());
                        }else{
                            m_odomTarget=LineupConstants.fiducialBlueLeftPoses[currentFiducial-6]; 
                            Translation2d flippedPoint=FlippingUtil.flipFieldPosition(m_odomTarget.getTranslation());
                            m_odomTarget=new Pose2d(flippedPoint.getX(),flippedPoint.getY(),m_odomTarget.getRotation());
                        }
                    } 
            
                    Command newCommand=m_swerve.createPathCommand(autoPlan.CreateOdomLineUpPath(m_swerve.getPose2d(),m_odomTarget));
                    CommandScheduler.getInstance().schedule(newCommand);

                    m_currentState=States.WAITFORLINEUP;
                }catch(Exception e){
                    m_currentState=States.END;
                }
            }else{
                m_startTime=Timer.getFPGATimestamp();

                m_currentState=States.END;
            }
        break;
        case WAITFORLINEUP:
            Pose2d currentPose=m_swerve.getPose2d();

            if(currentPose.getX()-LineupConstants.lineUpDeadband<m_odomTarget.getX()&&currentPose.getX()+LineupConstants.lineUpDeadband>m_odomTarget.getX()&&
                currentPose.getY()-LineupConstants.lineUpDeadband<m_odomTarget.getY()&&currentPose.getY()+LineupConstants.lineUpDeadband>m_odomTarget.getY()){
                m_currentState=States.ALGAE;
            }

            if(m_startTime+LineupConstants.maxWaitTime<Timer.getFPGATimestamp()){
                m_currentState=States.END;
            }
        break;
        case ALGAE:
            m_dunkin.algeaToggle(DunkinDonutConstants.autoScoreAlgaeSpeed);
            m_dunkin.toggleLocalPid(m_algaePos);

            m_currentState=States.ELEVATOR;
        break;
        case ELEVATOR:
            m_elevator.set_referance(m_elevatorFPos);

            m_currentState=States.WAITFORELEVATOR;
        break;
        case WAITFORELEVATOR:
            double elevatorPos=m_elevator.getRelativePos();

            if(m_elevator.getRelativePos()>elevatorPos-ElevatorConstants.posDeadband&&m_elevator.getRelativePos()<elevatorPos+ElevatorConstants.posDeadband){
                m_startTime=Timer.getFPGATimestamp();
                m_currentState=States.CORAL;
            }
        break;
        case CORAL:
            m_dunkin.runCoralMotor(DunkinDonutConstants.autoScoreCoralSpeed);

            if(DunkinDonutConstants.autoTimeToRun+m_startTime<Timer.getFPGATimestamp()){
                m_dunkin.stopCoralMotor();
                m_currentState=States.RETRACT;
            }
        break;
        case RETRACT:
            m_elevator.set_referance(ElevatorConstants.elevatorLowLimit);

            m_currentState=States.END;
        break;
        case END:
            returnValue=true;
        }

        return returnValue;
    }
}
