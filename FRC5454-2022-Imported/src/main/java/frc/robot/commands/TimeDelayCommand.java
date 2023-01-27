package frc.robot.commands;


import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class TimeDelayCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
 
  private boolean m_timerStarted=false;
  private double m_timeStart;
  private double m_timeEnd;
  private double m_timeDelay=0;
  
  /**
    * @param targetSpeed The speed we are setting in execute
   */
  public TimeDelayCommand(double timeDelay) {
    m_timeDelay=timeDelay;
    
  }
    // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
     if (!m_timerStarted){
      m_timerStarted=true;
      m_timeStart=Timer.getFPGATimestamp();
      m_timeEnd=m_timeStart+m_timeDelay;
      
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(final boolean interrupted) {
  // Release when button is released 
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    System.out.println(Timer.getFPGATimestamp() +  " --" + m_timeEnd);
    if(Timer.getFPGATimestamp()>=m_timeEnd){
      return true;
    }
      else {
      return false;
    }
  }
}

