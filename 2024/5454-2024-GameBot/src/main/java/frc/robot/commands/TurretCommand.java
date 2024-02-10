package frc.robot.commands;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.TurretConstants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.TurretSubsystem;

public class TurretCommand extends Command{
  private TurretSubsystem m_turret;
  
  private double m_speed;

  private double m_turretPos;
  private boolean m_isRunning;

  
  public TurretCommand(TurretSubsystem turret,double speed){
    m_turret=turret;
    m_speed=speed;
    m_isRunning=false;
    addRequirements(m_turret);
  }

  @Override
  public void execute(){
    m_isRunning=true;
    m_turret.ResetPIDReference();
  }

  @Override
  public void end(boolean interrupted){
    m_turret.stop();
    m_isRunning=false;
    Logger.recordOutput("Turret/TurretCommand",m_isRunning);

  }

  @Override
  public boolean isFinished(){
    Logger.recordOutput("Turret/TurretCommand",m_isRunning);

    m_turret.RunCheckLimits(m_speed);
  
    return false;
  }
}
