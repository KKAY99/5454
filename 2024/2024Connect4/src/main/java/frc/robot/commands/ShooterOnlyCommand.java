package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterOnlyCommand extends Command {
  private ShooterSubsystem m_Shooter;

  private double m_speed;
 
  public ShooterOnlyCommand(ShooterSubsystem shooter,double speed){
    m_Shooter=shooter;
    m_speed=speed;

  }

  @Override
  public void end(boolean interrupted){
    System.out.println("Stop Shooting");
    m_Shooter.StopShootingMotors();
  }

  public void execute(){
    System.out.println("Shooting at " + m_speed);       
    m_Shooter.RunShootingMotorsOnly(m_speed);

  }
  @Override
  public boolean isFinished(){
    
    return false;
  }
}
