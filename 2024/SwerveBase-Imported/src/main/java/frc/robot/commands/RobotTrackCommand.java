package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.Swerve;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.utilities.Limelight;
import frc.robot.subsystems.TurretSubsystem;

public class RobotTrackCommand extends Command {
  private Limelight m_limeLight;
  private TurretSubsystem m_turret;
  private boolean m_runLeft=true;
  private boolean m_runRight=false;

  public RobotTrackCommand(Limelight limeLight,TurretSubsystem turret){
    m_limeLight=limeLight;
    m_turret=turret;
  }

  @Override
  public void end(boolean interrupted){
    m_turret.stop();
  }

  @Override
  public boolean isFinished(){
    if(m_limeLight.isTargetAvailible()){
      double speed=0;
      if(Math.abs(m_limeLight.getXRaw())<Constants.LimeLightValues.m_closeXCheck){
        speed=Constants.LimeLightValues.limeLightTrackSpeed1;
      }else if(Math.abs(m_limeLight.getXRaw())<Constants.LimeLightValues.m_medXCheck){
        speed=Constants.LimeLightValues.limeLightTrackSpeed2;
      }else if(Math.abs(m_limeLight.getXRaw())<Constants.LimeLightValues.m_farXCheck){
        speed=Constants.LimeLightValues.limeLightTrackSpeed3;
      }else{
        speed=Constants.LimeLightValues.limeLightTrackSpeed4;
      }

    if(m_limeLight.getXRaw()<0){
      m_turret.RunCheckLimits(speed);
    }else if(m_limeLight.getXRaw()>0){
      m_turret.RunCheckLimits(-speed);
    }else{
      m_turret.stop();
      return true;
    }
    }else{
      if(m_runLeft){
        m_turret.RunCheckLimits(-Constants.LimeLightValues.limeLightTrackSpeed4);
        if(m_turret.IsAtLeftLimit()){
          m_runRight=true;
          m_runLeft=false;
        }
      }
      
      if(m_runRight){
        m_turret.RunCheckLimits(Constants.LimeLightValues.limeLightTrackSpeed4);
        if(m_turret.IsAtRightLimit()){
          m_runRight=false;
          m_runLeft=true;
        }
      }
    }
    return false;
  }
}
