package frc.robot.commands;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.*;
import frc.robot.Constants;
import frc.robot.Constants.swerveDrive;

/** An example command that uses an example subsystem. */
public class AutoMoveTimeCommand extends Command {
  private final DrivetrainSubsystem m_drive;
  private final double m_direction;
  private final double m_time;
  private final double m_rcw;
  private double m_startTime=0;
  private double m_speed=0;

  private enum STATES{
    STARTTIME,MOVE,END
  }

  private STATES m_currentState=STATES.STARTTIME;

  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})    
  public AutoMoveTimeCommand(DrivetrainSubsystem subsystem,double direction,double time,double speed) {
    m_drive=subsystem;
    m_direction=direction;
    m_speed=speed;
    m_time=time;
    m_rcw=0;
  }

   public AutoMoveTimeCommand(DrivetrainSubsystem subsystem,double direction,double rcw,double time,double speed) {
    m_drive=subsystem;
    m_direction=direction;
    m_speed=speed;
    m_time=time;
    m_rcw=rcw;
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_currentState=STATES.STARTTIME;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if(interrupted){
      m_drive.stopAutoDrive();
    } 
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    boolean returnValue=false;

    switch(m_currentState){
      case STARTTIME:
        m_startTime=Timer.getFPGATimestamp();
        m_currentState=STATES.MOVE;
      break;
      case MOVE:
        m_drive.moveGyroNodistance(m_direction, m_direction, m_speed);
      
        if(m_startTime+m_time<Timer.getFPGATimestamp()){
          m_currentState=STATES.END;
        }

      break;
      case END:
        m_drive.stopAutoDrive();
        returnValue=true;
    }

   return returnValue;
  }
}

