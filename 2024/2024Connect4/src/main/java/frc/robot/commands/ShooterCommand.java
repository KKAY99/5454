package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterCommand extends Command {
  private ShooterSubsystem m_Shooter;

  private double m_speed;

  public ShooterCommand(ShooterSubsystem shooter,double speed){
    m_Shooter=shooter;
    m_speed=speed;
    addRequirements(shooter);
  }

  @Override
  public void end(boolean interrupted){
    System.out.println("Stop Shooting");
    m_Shooter.StopShootingMotors();
  }

  public void execute(){
    System.out.println("Shooting at " + m_speed);       
    m_Shooter.RunShootingMotors(m_speed);

  }
  @Override
  public boolean isFinished(){
    
    return false;
  }
}
