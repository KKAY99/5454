package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DunkinDonutConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.LimeLightValues;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.utilities.Limelight;
import java.util.function.Supplier;

import com.ctre.phoenix6.controls.compound.Diff_TorqueCurrentFOC_Position;

import frc.robot.utilities.ObsidianPID;
import frc.robot.subsystems.DunkinDonutSubsystem;

public class ApriltagLineupCommand extends Command {
  private CommandSwerveDrivetrain m_swerve;
  private DunkinDonutSubsystem m_dunkin;

  private Limelight m_leftLimelight;
  private Limelight m_rightLimelight;

  private ObsidianPID m_drivePID;
  private ObsidianPID m_strafePID;

  private Supplier<Boolean> m_isRightLineup;

  private double m_startTime;

  private enum States{
    DRIVEFORWARDS,MOVECLAW,CHECKFORTARGET,LEFTLINEUP,RIGHTLINEUP,END
  };

  private States m_currentState = States.CHECKFORTARGET;

  public ApriltagLineupCommand(CommandSwerveDrivetrain swerve,DunkinDonutSubsystem dunkin,Limelight leftLimelight,Limelight rightLimelight,Supplier<Boolean> isRightLineup) {
    m_swerve=swerve;
    m_dunkin=dunkin;
    m_leftLimelight=leftLimelight;
    m_rightLimelight=rightLimelight;

    m_isRightLineup=isRightLineup;

    m_leftLimelight.setTargetHeight(LimeLightValues.reefAprilTagHeight);
    m_rightLimelight.setTargetHeight(LimeLightValues.reefAprilTagHeight);

    m_strafePID=new ObsidianPID(LimeLightValues.strafeP,LimeLightValues.strafeI,LimeLightValues.strafeD,LimeLightValues.strafeMaxAndMin,-LimeLightValues.strafeMaxAndMin);
    m_strafePID.setInputGain(LimeLightValues.strafeInputGain);

    m_drivePID=new ObsidianPID(LimeLightValues.driveP,LimeLightValues.driveI,LimeLightValues.driveD,LimeLightValues.driveMaxAndMin,-LimeLightValues.driveMaxAndMin);
    m_drivePID.setInputGain(LimeLightValues.driveInputGain);

    addRequirements(m_swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_currentState = States.MOVECLAW;
    m_startTime=Timer.getFPGATimestamp();
    //System.out.println("Command started");
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted){
    m_currentState = States.MOVECLAW;  
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    boolean returnValue=false;
    double distance=0;
    double rawX=0;
    double x=0;
    double strafeFlipValue=0;
    double strafe=0;
    double driveFlipValue=0;
    double drive=0;

    switch(m_currentState){
      case MOVECLAW:
      m_dunkin.resetShouldRunPID();
      m_dunkin.toggleLocalPid(DunkinDonutConstants.outOfLimelightVisionPos);

      m_currentState=States.DRIVEFORWARDS;
      break;
      case DRIVEFORWARDS:
        /*if(m_leftLimelight.isAnyTargetAvailable()||m_rightLimelight.isAnyTargetAvailable()){
          if(m_leftLimelight.isAnyTargetAvailable()&&!m_rightLimelight.isAnyTargetAvailable()){
            distance=Math.abs(m_leftLimelight.getDistance());
            driveFlipValue=distance/m_leftLimelight.getDistance();
            drive=(m_drivePID.calculatePercentOutput(distance,LimeLightValues.driveTargetDistanceLeft));
          }else{
            distance=Math.abs(m_rightLimelight.getDistance());
            driveFlipValue=-(distance/m_rightLimelight.getDistance());
            drive=-(m_drivePID.calculatePercentOutput(distance,LimeLightValues.driveTargetDistanceRight));
          }

          if(m_leftLimelight.isAnyTargetAvailable()&&!m_rightLimelight.isAnyTargetAvailable()&&LimeLightValues.driveTargetDistanceLeft-LimeLightValues.driveDeadBand<distance&&
                                                                                                LimeLightValues.driveTargetDistanceLeft+LimeLightValues.driveDeadBand>distance){

            m_currentState=States.CHECKFORTARGET;
          }else if(m_rightLimelight.isAnyTargetAvailable()&&LimeLightValues.driveTargetDistanceRight-LimeLightValues.driveDeadBand<distance&&
                                                            LimeLightValues.driveTargetDistanceRight+LimeLightValues.driveDeadBand>distance){
            m_currentState=States.CHECKFORTARGET;                                                 
          }else{
            m_swerve.drive(drive*driveFlipValue,0,0);
          }

        }else{
          m_currentState=States.END;
        }*/
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

        strafe=-m_strafePID.calculatePercentOutput(x,0);

        if(x<LimeLightValues.leftLineupXDeadband&&m_isRightLineup.get()){
          m_swerve.drive(0,0,0);
          m_currentState=States.END;
        }else{
          strafe=(m_isRightLineup.get())?strafe:-0.3;
          strafeFlipValue=(m_isRightLineup.get())?strafeFlipValue:1;
          m_swerve.drive(0,strafe*strafeFlipValue,0);
        }

        if(m_rightLimelight.isAnyTargetAvailable()&&!m_isRightLineup.get()){
          m_currentState=States.RIGHTLINEUP;
        }else if(!m_rightLimelight.isAnyTargetAvailable()&&!m_leftLimelight.isAnyTargetAvailable()){
          m_currentState=States.END;
        }
      break;
      case RIGHTLINEUP:
        rawX=m_rightLimelight.getX();
        x=Math.abs(m_rightLimelight.getX());
        strafeFlipValue=x/rawX;

        strafe=-m_strafePID.calculatePercentOutput(x,0);

        if(x<LimeLightValues.rightLineupXDeadband&&!m_isRightLineup.get()){
          m_swerve.drive(0,0,0);
          m_currentState=States.END;
        }else{
          strafe=(!m_isRightLineup.get())?strafe:0.3;
          strafeFlipValue=(!m_isRightLineup.get())?strafeFlipValue:1;
          m_swerve.drive(0,strafe*strafeFlipValue,0);
        }

        if(m_leftLimelight.isAnyTargetAvailable()&&m_isRightLineup.get()){
          m_currentState=States.LEFTLINEUP;
        }else if(!m_rightLimelight.isAnyTargetAvailable()&&!m_leftLimelight.isAnyTargetAvailable()){
          m_currentState=States.END;
        }
      break;
      case END:
        returnValue=true;
    }

    System.out.println("CURRENT PID OUTPUT"+strafe);

    return returnValue;
  }
}
