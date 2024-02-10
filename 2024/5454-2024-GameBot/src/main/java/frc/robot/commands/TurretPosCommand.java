package frc.robot.commands;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.TurretConstants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.TurretSubsystem;

public class TurretPosCommand extends Command{
  private TurretSubsystem m_turret;
  
  private double m_turretPos;
  private double m_timeOut;
  private double m_startTime;
  private double m_deadband;
  private boolean m_isRunning;

  public TurretPosCommand(TurretSubsystem turret,double pos,double timeout,double posCheckBand){
    m_turret=turret;   
    m_turretPos=pos;
    m_timeOut=timeout;
    m_deadband=posCheckBand;
    m_isRunning=false;
    addRequirements(m_turret);
    }

  @Override
  public void execute(){
    m_startTime=Timer.getFPGATimestamp();
    m_turret.TurretSetReference(m_turretPos);
    m_isRunning=true;
  }

  @Override
  public void end(boolean interrupted){
    m_turret.ResetPIDReference(); 
    m_turret.stop();
    m_isRunning=false;
    Logger.recordOutput("Turret/TurretPosCommand",m_isRunning);

  
  }
 

  @Override
  public boolean isFinished(){
    Logger.recordOutput("Turret/TurretPosCommand",m_isRunning);

    boolean hasTimedOut=(Timer.getFPGATimestamp()>(m_startTime+m_timeOut));
    boolean hasreachedPosition=m_turret.isAtPosition(m_turretPos,m_deadband);
    if(hasTimedOut||hasreachedPosition) {
      return true;
    }else{
      return false;
    }
    
  }
}
