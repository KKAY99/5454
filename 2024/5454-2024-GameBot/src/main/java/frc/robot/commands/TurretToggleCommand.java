package frc.robot.commands;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.TurretConstants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.TurretSubsystem;

public class TurretToggleCommand extends Command{
  private TurretSubsystem m_turret;
  
  private double m_turretPos;
  private double m_timeOut;
  private double m_startTime;
  private double m_deadband;
  private double kTurretToggleDeadband=5;
  private boolean m_isRunning;
  private boolean m_shouldGo90;

  public TurretToggleCommand(TurretSubsystem turret,double timeout){
    m_turret=turret;   
    m_timeOut=timeout;
    m_isRunning=false;
    addRequirements(m_turret);
    }

  @Override
  public void execute(){
    m_startTime=Timer.getFPGATimestamp();
    m_isRunning=true;

    if(m_turret.GetEncoderValue()<TurretConstants.turret90Pos-kTurretToggleDeadband&&m_turret.GetEncoderValue()>-18.5||m_turret.GetEncoderValue()>TurretConstants.turret90Pos+kTurretToggleDeadband){
      m_shouldGo90=true;
      System.out.println("Going To 90 1");

    }else if(m_turret.GetEncoderValue()>TurretConstants.turret90Pos-kTurretToggleDeadband&&m_turret.GetEncoderValue()<TurretConstants.turret90Pos+kTurretToggleDeadband){
      m_shouldGo90=false;
      System.out.println("Going To 0 1");

    }else if(m_turret.GetEncoderValue()<TurretConstants.turretStraightPos-kTurretToggleDeadband||m_turret.GetEncoderValue()>TurretConstants.turretStraightPos+kTurretToggleDeadband&&
    m_turret.GetEncoderValue()<-18.5){
      m_shouldGo90=false;
      System.out.println("Going To 90 2");

    }else if(m_turret.GetEncoderValue()>TurretConstants.turretStraightPos-kTurretToggleDeadband&&m_turret.GetEncoderValue()<TurretConstants.turretStraightPos+kTurretToggleDeadband){
      m_shouldGo90=true;
      System.out.println("Going To 0 2");

    }

    if(m_shouldGo90){
      m_turret.TurretSetReference(TurretConstants.turret90Pos);
    }else{
      m_turret.TurretSetReference(TurretConstants.turretStraightPos);
    }

  }

  @Override
  public void end(boolean interrupted){
   // m_turret.ResetPIDReference(); 
   // m_turret.stop();
    m_isRunning=false;
    Logger.recordOutput("Turret/TurretPosCommand",m_isRunning);

  
  }
  
  @Override
  public boolean isFinished(){
    return true;
  }
 
}
