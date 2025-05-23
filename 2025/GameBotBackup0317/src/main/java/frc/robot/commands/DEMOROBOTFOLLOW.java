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

public class DEMOROBOTFOLLOW extends Command {
  private CommandSwerveDrivetrain m_swerve;
  private DunkinDonutSubsystem m_dunkin;

  private Limelight m_leftLimelight;
  private Limelight m_rightLimelight;

  private ObsidianPID m_drivePID;
  private ObsidianPID m_rotPID;

  private enum States{
    MOVECLAW,SEARCH,FOLLOW,END
  };

  private States m_currentState = States.MOVECLAW;

  public DEMOROBOTFOLLOW(CommandSwerveDrivetrain swerve,DunkinDonutSubsystem dunkin,Limelight leftLimelight,Limelight rightLimelight) {
    m_swerve=swerve;
    m_dunkin=dunkin;
    m_leftLimelight=leftLimelight;
    m_rightLimelight=rightLimelight;

    m_leftLimelight.setTargetHeight(LimeLightValues.reefAprilTagHeight);
    m_rightLimelight.setTargetHeight(LimeLightValues.reefAprilTagHeight);

    m_rotPID=new ObsidianPID(LimeLightValues.rotP,LimeLightValues.rotI,LimeLightValues.rotD,LimeLightValues.rotMaxAndMin,-LimeLightValues.rotMaxAndMin);
    m_rotPID.setInputGain(LimeLightValues.rotInputGain);

    m_drivePID=new ObsidianPID(LimeLightValues.driveP,LimeLightValues.driveI,LimeLightValues.driveD,LimeLightValues.driveMaxAndMin,-LimeLightValues.driveMaxAndMin);
    m_drivePID.setInputGain(LimeLightValues.driveInputGain);

    addRequirements(m_swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_currentState = States.MOVECLAW;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted){
    m_swerve.drive(0,0,0);
    m_currentState = States.MOVECLAW;  
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    boolean returnValue=false;
    double distance=0;
    double rawX=0;
    double x=0;
    double rotFlipValue=0;
    double rot=0;
    double drive=0;

    switch(m_currentState){
      case MOVECLAW:
        m_dunkin.resetShouldRunPID();
        m_dunkin.toggleLocalPid(DunkinDonutConstants.outOfLimelightVisionPos);

        if(!m_leftLimelight.isAnyTargetAvailable()&&!!m_rightLimelight.isAnyTargetAvailable()){
          m_currentState=States.SEARCH;
        }else{
          m_currentState=States.FOLLOW;
        }
      break;
      case FOLLOW:
        if(m_leftLimelight.isAnyTargetAvailable()&&!m_rightLimelight.isAnyTargetAvailable()){
          rawX=m_leftLimelight.getX();
          x=Math.abs(m_leftLimelight.getX());
          distance=Math.abs(m_leftLimelight.getDistance());
          rotFlipValue=x/rawX;
           
          rot=-m_rotPID.calculatePercentOutput(x,0);
          drive=-(m_drivePID.calculatePercentOutput(distance,LimeLightValues.driveTargetDistanceLeft));

        }else if(m_rightLimelight.isAnyTargetAvailable()&&!m_leftLimelight.isAnyTargetAvailable()||m_rightLimelight.isAnyTargetAvailable()){
          rawX=m_rightLimelight.getX();
          x=Math.abs(m_rightLimelight.getX());
          distance=Math.abs(m_rightLimelight.getDistance());
          rotFlipValue=x/rawX;

          rot=m_rotPID.calculatePercentOutput(x,0);
          drive=-(m_drivePID.calculatePercentOutput(distance,LimeLightValues.driveTargetDistanceRight));

        }else{
          m_currentState=States.SEARCH;
        }

        m_swerve.drive(drive,0,rot*rotFlipValue);
      break;
      case SEARCH:
        if(m_leftLimelight.isAnyTargetAvailable()||m_rightLimelight.isAnyTargetAvailable()){
          m_swerve.drive(0,0,0);

          m_currentState=States.FOLLOW;
        }else{
          m_swerve.drive(0,0,0.3);
        }
      break;
      case END:
    }

    return returnValue;
  }
}
