package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterCommand extends Command {
  private ShooterSubsystem m_Shooter;

  private double m_speed;

  public ShooterCommand(ShooterSubsystem shooter,double speed){
    m_Shooter=shooter;
  }

  @Override
  public void end(boolean interrupted){
    m_Shooter.StopShootingMotors();
  }

  @Override
  public boolean isFinished(){
    m_Shooter.RunShootingMotors(m_speed);

    return false;
  }
}
