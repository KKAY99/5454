package frc.robot.commands;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

public class DelayCommand extends Command {
  private double m_timeToWait;
  private double m_startTime;

  public DelayCommand(double timeToWait){
    m_timeToWait=timeToWait;
  }

  @Override
  public void initialize(){
    m_startTime=Timer.getFPGATimestamp();
  }

  @Override
  public boolean isFinished() {
    return (m_startTime+m_timeToWait<Timer.getFPGATimestamp());
  }
}
