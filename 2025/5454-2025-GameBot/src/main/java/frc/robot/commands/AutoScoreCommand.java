package frc.robot.commands;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.util.FlippingUtil;
import com.revrobotics.spark.ClosedLoopSlot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.DunkinDonutConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.LEDStates;
import frc.robot.Constants.ElevatorConstants.ElevatorScoreLevel;
import frc.robot.Constants.LimeLightValues;
import frc.robot.Constants.LineupConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.DunkinDonutSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.utilities.AutoPlanner;
import frc.robot.utilities.Leds;
import frc.robot.utilities.Limelight;
import frc.robot.utilities.ObsidianPID;
import frc.robot.Constants.LedConstants;

public class AutoScoreCommand extends Command{
    public final Leds m_LEDS=new Leds(LedConstants.LedCanID,LedConstants.LedCount);

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
    private GenericEntry m_p;
    private GenericEntry m_i;
    private GenericEntry m_d;
    private GenericEntry m_max;
    private GenericEntry m_min;
    private GenericEntry m_inputGain;
    private double m_elevatorIPos;
    private double m_elevatorFPos;
    private double m_elevatorAlgaePos;
    private double m_algaePos;
    private double m_startTime;
    private double m_endTime;
    private double m_leftPIDOutput;
    private double m_rightPIDOutput;
    private double m_suspendDriveSpeed=0;
    private double m_startingHeartBeat;

    private Supplier<Boolean> m_isRightLineup;
    private Supplier<Boolean> m_doAlgae;
    private boolean m_isManual = false;
    private boolean m_shouldRunAlgae;
    private boolean m_startedCoral;
    private boolean m_isRunning;
    private boolean m_isInDeadBand;
    private boolean m_hasDrivenTwice=false;
    private boolean m_hasSetDriveTimer=false;
    
    private enum States{
        ISMANUALORAUTO,PRESCOREELEV,WAITFORCLAWROTATE,DRIVEFORWARDS,CHECKFORTARGET,LEFTLINEUP,RIGHTLINEUP,ALGAE,ALGAEGRAB,WAITFORALGAEGRAB,ALGAERETRACT,ELEVATOR,
        WAITFORELEVATOR,CORAL,ALGAEDRIVEBACK,RETRACT,WAITFORRETRACT,END
    }

    private States m_currentState=States.ISMANUALORAUTO;

    public AutoScoreCommand(CommandSwerveDrivetrain swerve,ElevatorSubsystem elevator,DunkinDonutSubsystem dunkin,Supplier<ElevatorScoreLevel> scorelevel,Supplier<Boolean> doAlgae,boolean isManualLineup){
        m_swerve=swerve;
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

        addRequirements(m_swerve,m_elevator,m_dunkin);
    }

    public AutoScoreCommand(CommandSwerveDrivetrain swerve,ElevatorSubsystem elevator,DunkinDonutSubsystem dunkin,Supplier<ElevatorScoreLevel> scorelevel,
                            Limelight leftLimelight,Limelight rightLimelight,Supplier<Boolean> isRightLineup,Supplier<Boolean> doAlgae,
                            GenericEntry p,GenericEntry i,GenericEntry d,GenericEntry max,GenericEntry min,GenericEntry inputGain){
        m_swerve=swerve;
        m_elevator=elevator;
        m_dunkin=dunkin;

        m_leftLimelight=leftLimelight;
        m_rightLimelight=rightLimelight;

        m_scoreLevel=scorelevel;
        m_isRightLineup=isRightLineup;
        m_doAlgae=doAlgae;
        m_isManual=false;

        m_p=p;
        m_i=i;
        m_d=d;
        m_max=max;
        m_min=min;
        m_inputGain=inputGain;

        m_leftLimelight.setTargetHeight(LimeLightValues.reefAprilTagHeight);
        m_rightLimelight.setTargetHeight(LimeLightValues.reefAprilTagHeight);

        m_strafePIDRIGHT=new ObsidianPID(LimeLightValues.strafePRIGHT,LimeLightValues.strafeIRIGHT,LimeLightValues.strafeDRIGHT,LimeLightValues.strafeMaxAndMinRIGHT,-LimeLightValues.strafeMaxAndMinRIGHT);
        m_strafePIDRIGHT.setInputGain(LimeLightValues.strafeInputGainRIGHT);
        m_strafePIDLEFT=new ObsidianPID(LimeLightValues.strafePLEFT,LimeLightValues.strafeILEFT,LimeLightValues.strafeDLEFT,LimeLightValues.strafeMaxAndMinLEFT,-LimeLightValues.strafeMaxAndMinLEFT);
        m_strafePIDLEFT.setInputGain(LimeLightValues.strafeInputGainLEFT);

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
        m_dunkin.runCoralShootMotor(0);
        m_shouldRunAlgae=false;
        m_startedCoral=false;
        m_isRunning=true;
        m_hasDrivenTwice=false;
        m_hasSetDriveTimer=false;
        
        if(m_p!=null){
            m_strafePIDLEFT.setAllValues(m_p.getDouble(0),m_i.getDouble(0),m_d.getDouble(0),m_max.getDouble(0),m_min.getDouble(0),m_inputGain.getDouble(0));
            m_strafePIDRIGHT.setAllValues(m_p.getDouble(0),m_i.getDouble(0),m_d.getDouble(0),m_max.getDouble(0),m_min.getDouble(0),m_inputGain.getDouble(0));
        }

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
            if(m_isManual){
                if(m_doAlgae.get()){
                    m_shouldRunAlgae=true;
                }
            }else {
                if(m_doAlgae.get()&&!m_isRightLineup.get()){
                    m_shouldRunAlgae=true;
                }
            }
            break;
            case L4:
            m_elevatorFPos=ElevatorConstants.l4Pos;
            m_elevatorAlgaePos=ElevatorConstants.l4AlgaePos;
            m_elevatorIPos=ElevatorConstants.l3Pos;
            m_algaePos=DunkinDonutConstants.noGrabAlgaePos;
            m_algaePos=DunkinDonutConstants.algaeGrabPosL4;
            if(m_isManual){
                if(m_doAlgae.get()){
                    m_shouldRunAlgae=true;
                }
            }else {
                if(m_doAlgae.get()&&!m_isRightLineup.get()){
                    m_shouldRunAlgae=true;
                }
            }
            break; 
            case TEST:
            m_elevatorFPos=m_dashBoardPos.get();
            m_elevatorIPos= m_dashBoardPos.get();
            break;
        }
    }

    @Override
    public void end(boolean interrupted) {
        m_dunkin.stopCoralShootMotor();
        m_dunkin.stop_rotatemotor();
        m_dunkin.resetShouldRunPID();
        m_swerve.drive(0,0,0);
        m_LEDS.setLedState(LEDStates.TELEOP,false);
        m_isRunning=false;
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
                m_dunkin.resetShouldRunPID();
                m_dunkin.toggleLocalPid(DunkinDonutConstants.outOfLimelightVisionPos);
                m_currentState=States.WAITFORCLAWROTATE;
            }else{
                m_dunkin.resetShouldRunPID();
                m_dunkin.toggleLocalPid(DunkinDonutConstants.outOfLimelightVisionPos);
                m_currentState=States.WAITFORCLAWROTATE;
            }
        break;
        case PRESCOREELEV:
            m_elevator.set_referance(m_elevatorIPos);

            m_currentState=States.DRIVEFORWARDS;
        break;
        case WAITFORCLAWROTATE:
            double clawABSPos=m_dunkin.getAbsoluteEncoderPos();
            if(clawABSPos+DunkinDonutConstants.posClawDeadband>DunkinDonutConstants.outOfLimelightVisionPos&&clawABSPos-DunkinDonutConstants.posClawDeadband<DunkinDonutConstants.outOfLimelightVisionPos){
                m_startTime=Timer.getFPGATimestamp();
                m_currentState=States.PRESCOREELEV;
            }
        break;
        case DRIVEFORWARDS:
            m_endTime=m_startTime+LimeLightValues.driveTimeToRun;
            //System.out.println("Starting Time" + m_startTime + "End Time " + m_endTime );
            if(Timer.getFPGATimestamp()>m_endTime){
                m_swerve.brake();
                m_swerve.drive(0,0,0);
                //System.out.println("Ending Time" + Timer.getFPGATimestamp());
                if(m_isManual){
                    m_currentState=States.ELEVATOR;
                    m_swerve.drive(m_suspendDriveSpeed,0,0);
                }else{
                    m_currentState=States.CHECKFORTARGET;
                    m_swerve.drive(m_suspendDriveSpeed,0,0);
                }
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
                    m_currentState=States.RETRACT;
                }
            }else{
                if(m_rightLimelight.isAnyTargetAvailable()){
                    m_currentState=States.RIGHTLINEUP;
                }else if(m_leftLimelight.isAnyTargetAvailable()){
                    m_currentState=States.LEFTLINEUP;
                }else{
                    m_currentState=States.RETRACT;
                }
            }
        break;
        case LEFTLINEUP:
            rawX=m_leftLimelight.getX();
            x=Math.abs(m_leftLimelight.getX());
            strafeFlipValue=x/rawX;

            strafe=-m_strafePIDLEFT.calculatePercentOutput(x,0);

            if(x<LimeLightValues.leftLineupXDeadband&&m_isRightLineup.get()&&x!=0){
                //System.out.println("IN LEFT LIMELIGHT DEADBAND IS CURRENT LINEUP RIGHT: "+m_isRightLineup.get());
                if(!m_hasSetDriveTimer){
                    m_hasSetDriveTimer=true;
                    m_startTime=Timer.getFPGATimestamp();
                    m_endTime=m_startTime+LimeLightValues.driveTimeToRunLeft;
                }

                if(!m_isInDeadBand&&m_hasDrivenTwice){
                    m_startingHeartBeat=m_leftLimelight.getHeartBeat();
                    m_isInDeadBand=true;
                }

                if(m_startingHeartBeat+LimeLightValues.isInDeadBandHeartBeat<=m_leftLimelight.getHeartBeat()&&m_hasDrivenTwice){
                    m_swerve.drive(0,0,0);

                    if(m_startingHeartBeat+LimeLightValues.shouldEndHeartBeat<=m_leftLimelight.getHeartBeat()&&m_hasDrivenTwice){
                        m_swerve.drive(0,0,0);
                        m_swerve.brake();
                        m_currentState=States.ELEVATOR;
                    }
                }else if(m_hasDrivenTwice){
                    strafe=(m_isRightLineup.get())?-strafe:-LimeLightValues.strafeMaxAndMinLEFT;
                    strafeFlipValue=(m_isRightLineup.get())?strafeFlipValue:1;
                    strafe=(strafe<0)?MathUtil.clamp(strafe,-LimeLightValues.strafeMaxAndMinLEFT,-LimeLightValues.strafeClampMinLEFT):
                                    MathUtil.clamp(strafe,LimeLightValues.strafeClampMinLEFT,LimeLightValues.strafeMaxAndMinLEFT);
                    //System.out.println("In DEADBAND LEFT LINEUP STRAFING AT "+strafe+" SPEED");
                    m_swerve.drive(0,strafe*strafeFlipValue,0);

                }else if(Timer.getFPGATimestamp()>m_endTime&&!m_hasDrivenTwice&&m_hasSetDriveTimer){
                    m_swerve.drive(0,0,0);
                    m_swerve.brake();
                    m_hasDrivenTwice=true;
                }else if(!m_hasDrivenTwice&&m_hasSetDriveTimer){
                    m_swerve.drive(LimeLightValues.correctDriveSpeed,0,0);
                }
            }else{
                if(!m_hasSetDriveTimer||m_hasDrivenTwice){
                    m_isInDeadBand=false;
                    strafe=(m_isRightLineup.get())?-strafe:-LimeLightValues.strafeMaxAndMinLEFT;
                    strafeFlipValue=(m_isRightLineup.get())?strafeFlipValue:1;
                    strafe=(strafe<0)?MathUtil.clamp(strafe,-LimeLightValues.strafeMaxAndMinLEFT,-LimeLightValues.strafeClampMinLEFT):
                                    MathUtil.clamp(strafe,LimeLightValues.strafeClampMinLEFT,LimeLightValues.strafeMaxAndMinLEFT);
                    //System.out.println("LEFT LINEUP STRAFING AT "+strafe+" SPEED");
                    m_swerve.drive(0,strafe*strafeFlipValue,0);

                }if(Timer.getFPGATimestamp()>m_endTime&&!m_hasDrivenTwice&&m_hasSetDriveTimer){
                    m_swerve.drive(0,0,0);
                    m_swerve.brake();
                    m_hasDrivenTwice=true;
                }else if(!m_hasDrivenTwice&&m_hasSetDriveTimer){
                    m_swerve.drive(LimeLightValues.correctDriveSpeed,0,0);
                }
            }

            if(m_rightLimelight.isAnyTargetAvailable()&&!m_isRightLineup.get()){
                //System.out.println("SWAPPING TO RIGHT LINEUP: " + !m_isRightLineup.get());
                m_hasSetDriveTimer=false;
                m_hasDrivenTwice=false;
                m_currentState=States.RIGHTLINEUP;
            }else if(!m_rightLimelight.isAnyTargetAvailable()&&!m_leftLimelight.isAnyTargetAvailable()){
                //System.out.println("NO TARGETS ENDING COMMAND");
                m_currentState=States.RETRACT;
            }

            m_leftPIDOutput=strafe;
        break;
        case RIGHTLINEUP:
            rawX=m_rightLimelight.getX();
            x=Math.abs(m_rightLimelight.getX());
            strafeFlipValue=x/rawX;

            strafe=-m_strafePIDRIGHT.calculatePercentOutput(x,0);

            if(x<LimeLightValues.rightLineupXDeadband&&!m_isRightLineup.get()&&x!=0){
                //System.out.println("IN RIGHT LIMELIGHT DEADBAND IS CURRENT LINEUP LEFT: "+!m_isRightLineup.get());
                if(!m_hasSetDriveTimer){
                    m_hasSetDriveTimer=true;
                    m_startTime=Timer.getFPGATimestamp();
                    m_endTime=m_startTime+LimeLightValues.driveTimeToRunRight;
                }

                if(!m_isInDeadBand&&m_hasDrivenTwice){
                    m_startingHeartBeat=m_rightLimelight.getHeartBeat();
                    m_isInDeadBand=true;
                }

                if(m_startingHeartBeat+LimeLightValues.isInDeadBandHeartBeat<=m_rightLimelight.getHeartBeat()&&m_hasDrivenTwice){
                    m_swerve.drive(0,0,0);

                    if(m_startingHeartBeat+LimeLightValues.shouldEndHeartBeat<=m_rightLimelight.getHeartBeat()&&m_hasDrivenTwice){
                        m_swerve.drive(0,0,0);
                        m_swerve.brake();
                        m_currentState=States.ELEVATOR;
                    }
                }else if(m_hasDrivenTwice){
                    strafe=(!m_isRightLineup.get())?strafe:LimeLightValues.strafeMaxAndMinRIGHT;
                    strafeFlipValue=(!m_isRightLineup.get())?strafeFlipValue:1;
                    strafe=(strafe<0)?MathUtil.clamp(strafe,-LimeLightValues.strafeMaxAndMinRIGHT,-LimeLightValues.strafeClampMinRIGHT)
                                    :MathUtil.clamp(strafe,LimeLightValues.strafeClampMinRIGHT,LimeLightValues.strafeMaxAndMinRIGHT);
                    //System.out.println("IN DEADBAND RIGHT LINEUP STRAFING AT "+strafe+" SPEED");
                    m_swerve.drive(0,strafe*strafeFlipValue,0);

                }else if(Timer.getFPGATimestamp()>m_endTime&&!m_hasDrivenTwice&&m_hasSetDriveTimer){
                    m_swerve.drive(0,0,0);
                    m_swerve.brake();
                    m_hasDrivenTwice=true;
                }else if(!m_hasDrivenTwice&&m_hasSetDriveTimer){
                    m_swerve.drive(LimeLightValues.correctDriveSpeed,0,0);
                }
            }else{
                if(!m_hasSetDriveTimer||m_hasDrivenTwice){
                    m_isInDeadBand=false;
                    strafe=(!m_isRightLineup.get())?strafe:LimeLightValues.strafeMaxAndMinRIGHT;
                    strafeFlipValue=(!m_isRightLineup.get())?strafeFlipValue:1;
                    strafe=(strafe<0)?MathUtil.clamp(strafe,-LimeLightValues.strafeMaxAndMinRIGHT,-LimeLightValues.strafeClampMinRIGHT)
                                    :MathUtil.clamp(strafe,LimeLightValues.strafeClampMinRIGHT,LimeLightValues.strafeMaxAndMinRIGHT);
                    //System.out.println("RIGHT LINEUP STRAFING AT "+strafe+" SPEED");
                    m_swerve.drive(0,strafe*strafeFlipValue,0);

                }else if(Timer.getFPGATimestamp()>m_endTime&&!m_hasDrivenTwice&&m_hasSetDriveTimer){
                    m_swerve.drive(0,0,0);
                    m_swerve.brake();
                    m_hasDrivenTwice=true;
                }else if(!m_hasDrivenTwice&&m_hasSetDriveTimer){
                    m_swerve.drive(LimeLightValues.correctDriveSpeed,0,0);
                }
            }

            if(m_leftLimelight.isAnyTargetAvailable()&&m_isRightLineup.get()){
                //System.out.println("SWAPPING TO LEFT LINEUP: " + m_isRightLineup.get());
                m_hasSetDriveTimer=false;
                m_hasDrivenTwice=false;
                m_currentState=States.LEFTLINEUP;
            }else if(!m_rightLimelight.isAnyTargetAvailable()&&!m_leftLimelight.isAnyTargetAvailable()){
                //System.out.println("NO TARGETS ENDING COMMAND");
                m_currentState=States.RETRACT;
            }

            m_rightPIDOutput=strafe;
        break;
        case ELEVATOR:
            m_LEDS.setLedState(LEDStates.AUTOSCORING,true);
            m_elevator.set_referance(m_elevatorFPos,ClosedLoopSlot.kSlot0);
            if(m_shouldRunAlgae){
                m_dunkin.resetShouldRunPID();
                m_dunkin.toggleLocalPid(m_algaePos);
                m_dunkin.resetAlgeaToggle();
                m_dunkin.algeaToggle(DunkinDonutConstants.autoScoreAlgaeSpeed);
                m_startTime=Timer.getFPGATimestamp();
            }

            m_currentState=States.WAITFORELEVATOR;
        break;
        case WAITFORELEVATOR:
            elevatorPos=m_elevator.getRelativePos();

            if(Timer.getFPGATimestamp()>m_startTime+DunkinDonutConstants.algaeRotateDownTime){
                if(m_scoreLevel.get()==ElevatorScoreLevel.L4&&m_shouldRunAlgae){
                    m_dunkin.resetShouldRunPID();
                    m_dunkin.toggleLocalPid(DunkinDonutConstants.algaeMoveForScoringL4);
                }
            }

            if(elevatorPos>m_elevatorFPos-ElevatorConstants.posDeadband&&elevatorPos<m_elevatorFPos+ElevatorConstants.posDeadband){
                m_startTime=Timer.getFPGATimestamp();
                m_currentState=States.CORAL;
            }
        break;
        case CORAL:
            if(m_scoreLevel.get()==ElevatorScoreLevel.L4){
                m_dunkin.runCoralShootMotor(DunkinDonutConstants.autoScoreCoralSpeedL4);
            }else{
                m_dunkin.runCoralShootMotor(DunkinDonutConstants.autoScoreCoralSpeed);
            }

            if(DunkinDonutConstants.autoCoralTimeToRun+m_startTime<Timer.getFPGATimestamp()){
                m_dunkin.stopCoralShootMotor();
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
                m_dunkin.runAlgaeMotor(DunkinDonutConstants.keepAlgaeSpeed);
                m_swerve.drive(LimeLightValues.algaeDriveBackSpeed,0,0);
            }
        break;
        case RETRACT:
            if(m_shouldRunAlgae){
                m_elevator.set_referance(ElevatorConstants.elevAlgeaGrabRetractPos,ClosedLoopSlot.kSlot1);
            }else{
                m_elevator.set_referance(ElevatorConstants.elevatorLowLimit,ClosedLoopSlot.kSlot1);
            }

            m_currentState=States.WAITFORRETRACT;
        break;
        case WAITFORRETRACT:
            elevatorPos=m_elevator.getRelativePos();
            
            if(m_shouldRunAlgae){
                if(elevatorPos>ElevatorConstants.elevAlgeaGrabRetractPos-ElevatorConstants.posDeadband&&ElevatorConstants.elevAlgeaGrabRetractPos<elevatorPos+ElevatorConstants.posDeadband){
        
                    m_currentState=States.END;
                }
            }else{
                if(elevatorPos>ElevatorConstants.elevSafeRetractPos){

                    m_currentState=States.END;
                }
            }
        break;
        case END:
            returnValue=true;
        }
        try{
            Logger.recordOutput("Commands/AutoScore/IsRunning",m_isRunning);
            Logger.recordOutput("Commands/AutoScore/CurrentState",m_currentState);
            if(m_leftLimelight!=null&&m_rightLimelight!=null){
                Logger.recordOutput("Commands/AutoScore/LimelightLeftX",m_leftLimelight.getX());
                Logger.recordOutput("Commands/AutoScore/LimelightRightX",m_rightLimelight.getX());
                Logger.recordOutput("Commands/AutoScore/LimelightLeftAnyTargets",m_leftLimelight.isAnyTargetAvailable());
                Logger.recordOutput("Commands/AutoScore/LimelightRightAnyTargets",m_rightLimelight.isAnyTargetAvailable());
                Logger.recordOutput("Commands/AutoScore/LeftLineup",!m_isRightLineup.get());
                Logger.recordOutput("Commands/AutoScore/RightLineup",m_isRightLineup.get());
                Logger.recordOutput("Commands/AutoScore/RightPIDOutput",m_rightPIDOutput);
                Logger.recordOutput("Commands/AutoScore/LeftPIDOutput",m_leftPIDOutput);
                //System.out.println("AutoScore-LimelightLeftAnyTargets: "+m_leftLimelight.isAnyTargetAvailable());
                //System.out.println("AutoScore-LimelightRightAnyTargets: "+m_rightLimelight.isAnyTargetAvailable());
            }
            Logger.recordOutput("Commands/AutoScore/ElevatorPos",m_elevator.getRelativePos());
            Logger.recordOutput("Commands/AutoScore/ClawPos",m_dunkin.get_rotatemotorpos());
            Logger.recordOutput("Commands/AutoScore/DoAlgae",m_doAlgae.get());
            Logger.recordOutput("Commands/AutoScore/ShouldRunAlgae",m_shouldRunAlgae);
            Logger.recordOutput("Commands/AutoScore/ElevatorScoreLevel",m_scoreLevel.get());
            Logger.recordOutput("Commands/AutoScore/SwerveYSpeed",m_swerve.getChassisSpeeds().vyMetersPerSecond);
            Logger.recordOutput("Commands/AutoScore/SwerveXSpeed",m_swerve.getChassisSpeeds().vxMetersPerSecond);
            
            //System.out.println("AutoScore-CurrentState: "+m_currentState);
        } catch (Exception e){
            System.out.println("Logging Exception Error");
        }
        return returnValue;
    }
}
