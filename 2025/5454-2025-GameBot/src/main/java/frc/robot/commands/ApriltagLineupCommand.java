package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.LimeLightValues;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.utilities.Limelight;
import java.util.function.Supplier;
import frc.robot.utilities.ObsidianPID;

public class ApriltagLineupCommand extends Command {
  private CommandSwerveDrivetrain m_swerve;
  private Limelight m_leftLimelight;
  private Limelight m_rightLimelight;

  private ObsidianPID m_pid;

  private Supplier<Boolean> m_isRightLineup;

  private enum States{
    CHECKFORTARGET,LEFTLINEUP,RIGHTLINEUP,END
  };

  private States m_currentState = States.CHECKFORTARGET;

  public ApriltagLineupCommand(CommandSwerveDrivetrain swerve,Limelight leftLimelight,Limelight rightLimelight,Supplier<Boolean> isRightLineup) {
    m_swerve=swerve;
    m_leftLimelight=leftLimelight;
    m_rightLimelight=rightLimelight;

    m_isRightLineup=isRightLineup;

    m_leftLimelight.setTargetHeight(LimeLightValues.reefAprilTagHeight);
    m_rightLimelight.setTargetHeight(LimeLightValues.reefAprilTagHeight);

    m_pid=new ObsidianPID(LimeLightValues.P,LimeLightValues.I,LimeLightValues.D,LimeLightValues.maxAndMin,-LimeLightValues.maxAndMin);
    m_pid.setInputGain(LimeLightValues.inputGain);

    addRequirements(m_swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_currentState = States.CHECKFORTARGET;
    //System.out.println("Command started");
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted){
    m_currentState = States.CHECKFORTARGET;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    boolean returnValue=false;
    double rawX=0;
    double x=0;
    double strafeFlipValue=0;
    double strafe=0;

    switch(m_currentState){
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

        strafe=-m_pid.calculatePercentOutput(x,0);

        if(x<LimeLightValues.leftLineupXDeadband&&m_isRightLineup.get()){
          m_swerve.drive(0,0,0);
          m_currentState=States.END;
        }else{
          //TODO: FINISH WHEN GET ROBOT
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

        strafe=m_pid.calculatePercentOutput(x,0);

        if(x<LimeLightValues.rightLineupXDeadband&&!m_isRightLineup.get()){
          m_swerve.drive(0,0,0);
          m_currentState=States.END;
        }else{
          //TODO: FINISH WHEN GET ROBOT
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
