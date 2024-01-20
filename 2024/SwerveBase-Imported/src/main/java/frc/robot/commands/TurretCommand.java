package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.TurretSubsystem;

public class TurretCommand extends Command{
  private TurretSubsystem m_turret;

  private double m_speed;

  public TurretCommand(TurretSubsystem turret,double speed){
    m_turret=turret;
    m_speed=speed;
  }

  @Override
  public void end(boolean interrupted){
    m_turret.stop();
  }

  @Override
  public boolean isFinished(){
    m_turret.RunTurretMotor(m_speed);
    return false;
  }
}
