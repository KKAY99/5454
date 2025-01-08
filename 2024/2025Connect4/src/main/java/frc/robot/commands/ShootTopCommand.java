package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootTopCommand extends Command {
  private ShooterSubsystem m_Shooter;

  private double m_speed;

  public ShootTopCommand(ShooterSubsystem shooter,double speed){
    m_Shooter=shooter;
    m_speed=speed;
    addRequirements(shooter);
  }

  @Override
  public void end(boolean interrupted){
  }

  public void execute(){
    System.out.println("Shooting at " + m_speed);       
    m_Shooter.RunTopMotor(m_speed);

  }
  @Override
  public boolean isFinished(){
    
    return false;
  }
}
