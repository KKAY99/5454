package frc.robot.commands;
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

  public TurretPosCommand(TurretSubsystem turret,double pos,double timeout,double posCheckBand){
    m_turret=turret;   
    m_turretPos=pos;
    m_timeOut=timeout;
    m_deadband=posCheckBand;
    }

  @Override
  public void execute(){
    m_startTime=Timer.getFPGATimestamp();
    m_turret.TurretSetReference(m_turretPos);
  }

  @Override
  public void end(boolean interrupted){
    m_turret.ResetPIDReference(); 
    m_turret.stop();
  
  }

  @Override
  public boolean isFinished(){
    boolean hasTimedOut = (Timer.getFPGATimestamp()>(m_startTime+m_timeOut));
    boolean hasreachedPosition=m_turret.isAtPosition(m_turretPos,m_deadband);
    if(hasTimedOut || hasreachedPosition) {
      return true;
    }else{
      return false;
    }
    
  }
}
