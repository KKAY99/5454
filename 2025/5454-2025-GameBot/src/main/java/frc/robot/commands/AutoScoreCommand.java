package frc.robot.commands;
import java.util.function.Supplier;

import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.DunkinDonutConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.ElevatorConstants.ElevatorScoreLevel;
import frc.robot.Constants.LimeLightValues;
import frc.robot.Constants.LineupConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.DunkinDonutSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.utilities.AutoPlanner;
import frc.robot.utilities.Limelight;
import frc.robot.utilities.ObsidianPID;

public class AutoScoreCommand extends Command{
    private CommandSwerveDrivetrain m_swerve;
    private ElevatorSubsystem m_elevator;
    private DunkinDonutSubsystem m_dunkin;

    private Limelight m_leftLimelight;
    private Limelight m_rightLimelight;

    private ObsidianPID m_drivePID;
    private ObsidianPID m_strafePIDRIGHT;
    private ObsidianPID m_strafePIDLEFT;

    private Supplier<ElevatorScoreLevel> m_scoreLevel;

    private Pose2d m_odomTarget;

    public Supplier<Double> m_dashBoardPos;
    private double m_elevatorIPos;
    private double m_elevatorFPos;
    private double m_elevatorAlgaePos;
    private double m_algaePos;
    private double m_startTime;

    private Supplier<Boolean> m_isRightLineup;
    private Supplier<Boolean> m_doAlgae;
    private boolean m_isManual = false;
    private boolean m_shouldRunAlgae;
    private boolean m_startedCoral;
    
    private enum States{
        ISMANUALORAUTO,PRESCOREELEV,DRIVEFORWARDS,CHECKFORTARGET,LEFTLINEUP,RIGHTLINEUP,ALGAE,ALGAEGRAB,WAITFORALGAEGRAB,ALGAERETRACT,ELEVATOR,
        WAITFORELEVATOR,CORAL,ALGAEDRIVEBACK,RETRACT,WAITFORRETRACT,END
    }

    private States m_currentState=States.ISMANUALORAUTO;

    public AutoScoreCommand(ElevatorSubsystem elevator,DunkinDonutSubsystem dunkin,Supplier<ElevatorScoreLevel> scorelevel,Supplier<Boolean> doAlgae){
        m_elevator=elevator;
        m_dunkin=dunkin;

        m_scoreLevel=scorelevel;
        m_isManual = true;
        m_doAlgae=doAlgae;

        addRequirements(m_elevator,m_dunkin);
    }

    public AutoScoreCommand(CommandSwerveDrivetrain swerve,ElevatorSubsystem elevator,DunkinDonutSubsystem dunkin,Supplier<ElevatorScoreLevel> scorelevel,
                            Limelight leftLimelight,Limelight rightLimelight,Supplier<Boolean> isRightLineup,Supplier<Boolean> doAlgae){
        m_swerve=swerve;
        m_elevator=elevator;
        m_dunkin=dunkin;

        m_leftLimelight=leftLimelight;
        m_rightLimelight=rightLimelight;

        m_scoreLevel=scorelevel;
        m_isRightLineup=isRightLineup;
        m_doAlgae=doAlgae;
        m_isManual=false;

        m_leftLimelight.setTargetHeight(LimeLightValues.reefAprilTagHeight);
        m_rightLimelight.setTargetHeight(LimeLightValues.reefAprilTagHeight);

        m_strafePIDRIGHT=new ObsidianPID(LimeLightValues.strafePRIGHT,LimeLightValues.strafeIRIGHT,LimeLightValues.strafeDRIGHT,LimeLightValues.strafeMaxAndMinRIGHT,-LimeLightValues.strafeMaxAndMinRIGHT);
        m_strafePIDRIGHT.setInputGain(LimeLightValues.strafeInputGainRIGHT);
        m_strafePIDLEFT=new ObsidianPID(LimeLightValues.strafePLEFT,LimeLightValues.strafeILEFT,LimeLightValues.strafeDLEFT,LimeLightValues.strafeMaxAndMinLEFT,-LimeLightValues.strafeMaxAndMinLEFT);
        m_strafePIDLEFT.setInputGain(LimeLightValues.strafeInputGainLEFT);

        m_drivePID=new ObsidianPID(LimeLightValues.driveP,LimeLightValues.driveI,LimeLightValues.driveD,LimeLightValues.driveMaxAndMin,-LimeLightValues.driveMaxAndMin);
        m_drivePID.setInputGain(LimeLightValues.driveInputGain);

        addRequirements(m_swerve,m_elevator,m_dunkin);
    }

    public AutoScoreCommand(CommandSwerveDrivetrain swerve,ElevatorSubsystem elevator,DunkinDonutSubsystem dunkin,Supplier<Double> elevPos,Supplier<Boolean> doAlgae){
        m_swerve=swerve;
        m_elevator=elevator;
        m_dunkin=dunkin;

        m_scoreLevel=()->ElevatorScoreLevel.TEST;
        m_dashBoardPos=elevPos;
        m_doAlgae=doAlgae;
        m_isManual=true;

        addRequirements(m_swerve,m_elevator,m_dunkin);
    }

    @Override
    public void initialize(){
        m_currentState=States.ISMANUALORAUTO;
        m_shouldRunAlgae=false;
        m_startedCoral=false;

        switch(m_scoreLevel.get()){
            case L1:
            m_elevatorFPos=ElevatorConstants.l1Pos;
            m_elevatorIPos=ElevatorConstants.l1Pos;
            m_algaePos=DunkinDonutConstants.noGrabAlgaePos;
            break;
            case L2:
            m_elevatorFPos=ElevatorConstants.l2Pos;
            m_elevatorIPos=ElevatorConstants.l2Pos;
            break;
            case L3:
            m_elevatorFPos=ElevatorConstants.l3Pos;
            m_elevatorAlgaePos=ElevatorConstants.l3AlgaePos;
            m_elevatorIPos=ElevatorConstants.l3Pos;
            m_algaePos=DunkinDonutConstants.algaeGrabPos;
            if(m_doAlgae.get()&&!m_isRightLineup.get()){
                m_shouldRunAlgae=true;
            }
            break;
            case L4:
            m_elevatorFPos=ElevatorConstants.l4Pos;
            m_elevatorAlgaePos=ElevatorConstants.l4AlgaePos;
            m_elevatorIPos=ElevatorConstants.l3Pos;
            m_algaePos=DunkinDonutConstants.noGrabAlgaePos;
            m_algaePos=DunkinDonutConstants.algaeGrabPos;
            if(m_doAlgae.get()&&!m_isRightLineup.get()){
                m_shouldRunAlgae=true;
            }
            break; 
            case TEST:
            m_elevatorFPos=m_dashBoardPos.get();
            m_elevatorIPos= m_dashBoardPos.get();
            break;
        }

        System.out.println("DOALGAE"+m_doAlgae.get());
        System.out.println("SHOULDRUNALGAE"+m_shouldRunAlgae);
    }

    @Override
    public void end(boolean interrupted) {
        m_dunkin.stopAlgeaMotor();
        m_dunkin.stopCoralMotor();
        m_dunkin.stop_rotatemotor();
        m_elevator.reset_referance();
        m_elevator.motor_stop();
        m_dunkin.resetShouldRunPID();
    }

    @Override
    public boolean isFinished(){
        boolean returnValue=false;
        double rawX=0;
        double x=0;
        double strafeFlipValue=0;
        double strafe=0;
        double elevatorPos;

        switch(m_currentState){
        case ISMANUALORAUTO:     
            if(m_isManual){
                m_currentState=States.ELEVATOR;
            }else{
                m_currentState=States.PRESCOREELEV;
            }
        break;
        case PRESCOREELEV:
            m_elevator.set_referance(m_elevatorIPos);
            m_dunkin.resetShouldRunPID();
            m_dunkin.toggleLocalPid(DunkinDonutConstants.outOfLimelightVisionPos);

            m_currentState=States.DRIVEFORWARDS;
        break;
        case DRIVEFORWARDS:
            if(m_startTime+LimeLightValues.driveTimeToRun<Timer.getFPGATimestamp()){
                m_swerve.drive(0,0,0);
                m_currentState=States.CHECKFORTARGET;
            }else{
                m_swerve.drive(LimeLightValues.lineUpDriveSpeed,0,0);
            }
        break;
        case CHECKFORTARGET:
            if(m_isRightLineup.get()){
                if(m_leftLimelight.isAnyTargetAvailable()){
                    m_currentState=States.LEFTLINEUP;
                }else if(m_rightLimelight.isAnyTargetAvailable()){
                    m_currentState=States.RIGHTLINEUP;
                }else{
                    m_currentState=States.END;
                }
            }else{
                if(m_rightLimelight.isAnyTargetAvailable()){
                    m_currentState=States.RIGHTLINEUP;
                }else if(m_leftLimelight.isAnyTargetAvailable()){
                    m_currentState=States.LEFTLINEUP;
                }else{
                    m_currentState=States.END;
                }
            }
        break;
        case LEFTLINEUP:
            rawX=m_leftLimelight.getX();
            x=Math.abs(m_leftLimelight.getX());
            strafeFlipValue=x/rawX;

            strafe=-m_strafePIDLEFT.calculatePercentOutput(x,0);

            if(x<LimeLightValues.leftLineupXDeadband&&m_isRightLineup.get()&&x!=0){
                m_swerve.drive(0,0,0);
                m_currentState=States.ELEVATOR;
            }else{
                strafe=(m_isRightLineup.get())?-strafe:-0.3;
                strafeFlipValue=(m_isRightLineup.get())?strafeFlipValue:1;
                m_swerve.drive(0,strafe*strafeFlipValue,0);
            }

            if(m_rightLimelight.isAnyTargetAvailable()&&!m_isRightLineup.get()){
                m_currentState=States.RIGHTLINEUP;
            }else if(!m_rightLimelight.isAnyTargetAvailable()&&!m_leftLimelight.isAnyTargetAvailable()){
                m_currentState=States.RETRACT;
            }
            System.out.println("CURRENT X:"+x);
            System.out.println("Current State:"+m_currentState.toString());
        break;
        case RIGHTLINEUP:
            rawX=m_rightLimelight.getX();
            x=Math.abs(m_rightLimelight.getX());
            strafeFlipValue=x/rawX;

            strafe=-m_strafePIDRIGHT.calculatePercentOutput(x,0);

            if(x<LimeLightValues.rightLineupXDeadband&&!m_isRightLineup.get()){
                m_swerve.drive(0,0,0);
                m_currentState=States.ELEVATOR;
            }else{
                strafe=(!m_isRightLineup.get())?strafe:0.3;
                strafeFlipValue=(!m_isRightLineup.get())?strafeFlipValue:1;
                m_swerve.drive(0,strafe*strafeFlipValue,0);
            }

            if(m_leftLimelight.isAnyTargetAvailable()&&m_isRightLineup.get()){
                m_dunkin.algeaToggle(DunkinDonutConstants.autoScoreAlgaeSpeed);
                m_currentState=States.LEFTLINEUP;
            }else if(!m_rightLimelight.isAnyTargetAvailable()&&!m_leftLimelight.isAnyTargetAvailable()){
                m_currentState=States.RETRACT;
            }

            System.out.println("CURRENT X:"+x);
            System.out.println("Current State:"+m_currentState.toString());
        break;
        case ELEVATOR:
            m_elevator.set_referance(m_elevatorFPos);
            if(m_shouldRunAlgae){
                m_dunkin.resetShouldRunPID();
                m_dunkin.toggleLocalPid(m_algaePos);
                m_dunkin.resetAlgeaToggle();
                m_dunkin.algeaToggle(DunkinDonutConstants.autoScoreAlgaeSpeed);
            }

            System.out.println("ELEVATOR SET REF");
            m_currentState=States.WAITFORELEVATOR;
        break;
        case WAITFORELEVATOR:
            elevatorPos=m_elevator.getRelativePos();

            if(elevatorPos>m_elevatorFPos-ElevatorConstants.posDeadband&&elevatorPos<m_elevatorFPos+ElevatorConstants.posDeadband){
                System.out.println("ELEVATOR IS AT POS");

                m_startTime=Timer.getFPGATimestamp();
                m_currentState=States.CORAL;
            }
        break;
        case CORAL:
            m_dunkin.runCoralMotor(DunkinDonutConstants.autoScoreCoralSpeed);

            if(DunkinDonutConstants.autoCoralTimeToRun+m_startTime<Timer.getFPGATimestamp()){
                System.out.println("CORAL FINISHED");

                m_dunkin.stopCoralMotor();
                m_currentState=m_shouldRunAlgae?States.ALGAEGRAB:States.RETRACT;
            }

            if(m_shouldRunAlgae&&!m_startedCoral){
                m_startedCoral=true;
            }
        break;
        case ALGAEGRAB:
            m_elevator.set_referance(m_elevatorAlgaePos);

            m_currentState=States.WAITFORALGAEGRAB;
        break;
        case WAITFORALGAEGRAB:
            elevatorPos=m_elevator.getRelativePos();

            if(elevatorPos>m_elevatorAlgaePos-ElevatorConstants.posDeadband&&elevatorPos<m_elevatorAlgaePos+ElevatorConstants.posDeadband){
                m_currentState=States.ALGAE;
            }
        break;
        case ALGAE:
            if(m_startTime+DunkinDonutConstants.autoScoreAlgaeRunTime<Timer.getFPGATimestamp()){
                m_currentState=States.ALGAERETRACT;
            }
        break;
        case ALGAERETRACT:
            m_dunkin.resetShouldRunPID();
            m_dunkin.algeaToggle(DunkinDonutConstants.autoScoreAlgaeSpeed);
            m_dunkin.toggleLocalPid(DunkinDonutConstants.algaeStowPos);

            m_startTime=Timer.getFPGATimestamp();
            m_currentState=States.ALGAEDRIVEBACK;
        break;
        case ALGAEDRIVEBACK:
            if(m_startTime+LimeLightValues.algaeDriveTimeToRun<Timer.getFPGATimestamp()){
                m_swerve.drive(0,0,0);
                m_currentState=States.RETRACT;
            }else{
                m_swerve.drive(LimeLightValues.alageDriveBackSpeed,0,0);
            }
        break;
        case RETRACT:
            if(m_shouldRunAlgae){
                m_elevator.set_referance(ElevatorConstants.elevAlgeaGrabRetractPos);
            }else{
                m_elevator.set_referance(ElevatorConstants.elevatorLowLimit);
            }
            System.out.println("ELEVATOR RETRACT");

            m_currentState=States.WAITFORRETRACT;
        break;
        case WAITFORRETRACT:
            elevatorPos=m_elevator.getRelativePos();
            
            if(m_shouldRunAlgae){
                if(elevatorPos>ElevatorConstants.elevAlgeaGrabRetractPos-ElevatorConstants.posDeadband&&ElevatorConstants.elevAlgeaGrabRetractPos<elevatorPos+ElevatorConstants.posDeadband){
                    System.out.println("ELEVATOR IS AT POS");
        
                    m_currentState=States.END;
                }
            }else{
                if(elevatorPos>ElevatorConstants.elevatorLowLimit-ElevatorConstants.posDeadband&&ElevatorConstants.elevatorLowLimit<elevatorPos+ElevatorConstants.posDeadband){
                    System.out.println("ELEVATOR IS AT POS");
        
                    m_currentState=States.END;
                }
            }
        break;
        case END:
            returnValue=true;
        }

        return returnValue;
    }
}
