package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class HardStopShooter extends Command{

  private ShooterSubsystem m_shooter;

  public HardStopShooter(ShooterSubsystem shooter){
    m_shooter=shooter;
  }

  @Override
  public void execute(){
    m_shooter.stopShooter();
    m_shooter.StopFeedRollers();
  }

  @Override
  public void end(boolean interrupted){}

  @Override
  public boolean isFinished(){
    return true;
  }

}
