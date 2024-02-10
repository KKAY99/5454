package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.TurretConstants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.TurretSubsystem;

public class TurretCommand extends Command{
  private TurretSubsystem m_turret;
  
  private double m_speed;

  private double m_turretPos;

  
  public TurretCommand(TurretSubsystem turret,double speed){
    m_turret=turret;
    m_speed=speed;
  
    addRequirements(m_turret);
  }

  @Override
  public void execute(){
    m_turret.ResetPIDReference();
  }

  @Override
  public void end(boolean interrupted){
    m_turret.stop();
  }

  @Override
  public boolean isFinished(){
   
    m_turret.RunCheckLimits(m_speed);
  
    return false;
  }
}
