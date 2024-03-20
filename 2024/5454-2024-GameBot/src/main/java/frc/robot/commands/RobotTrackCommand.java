package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.utilities.Limelight;

public class RobotTrackCommand extends Command{
  private Limelight m_limeLight;
  private TurretSubsystem m_turret;
  
  private boolean m_runLeft=true;
  private boolean m_runRight=false;

  public RobotTrackCommand(Limelight limeLight,TurretSubsystem turret){
    m_limeLight=limeLight;
    m_turret=turret;

    addRequirements(m_turret);
  }

  @Override
  public void end(boolean interrupted){
    //m_turret.TrackTarget(false);
    m_turret.stop();
  }

  @Override
  public boolean isFinished(){
    //m_turret.TrackTarget(true);
    
    return m_turret.IsOnTarget();

    //double speed=0;

   // double multiplier=m_limeLight.GetDistanceMultipler();

    /*switch(m_state){
    case TRACKING:
      if(m_limeLight.isTargetAvailible()){
        if(Math.abs(m_limeLight.getXRaw())<Constants.LimeLightValues.limeLightDeadBand*multiplier){
          speed=0;
        }else if(Math.abs(m_limeLight.getXRaw())<Constants.LimeLightValues.closerXCheck*multiplier){
          speed=Constants.LimeLightValues.limeLightTrackSpeed1;
        }else if(Math.abs(m_limeLight.getXRaw())<Constants.LimeLightValues.closeXCheck*multiplier){
          speed=Constants.LimeLightValues.limeLightTrackSpeed2;
        }else if(Math.abs(m_limeLight.getXRaw())<Constants.LimeLightValues.medXCheck*multiplier){
          speed=Constants.LimeLightValues.limeLightTrackSpeed3;
        }else if(Math.abs(m_limeLight.getXRaw())<Constants.LimeLightValues.farXCheck*multiplier){
          speed=Constants.LimeLightValues.limeLightTrackSpeed4;
        }else{
          speed=Constants.LimeLightValues.limeLightTrackSpeed5;
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
        if(speed>0){
          m_runRight=true;
          m_runLeft=false;
        }else if(speed<0){
          m_runLeft=true;
          m_runRight=false;
        }else{
          m_runRight=true;
          m_runLeft=false;
        }
        m_state=TrackingStates.SEARCHING;
      }

    break;
    case SEARCHING:
      if(!m_limeLight.isTargetAvailible()){
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
      }else{
        m_state=TrackingStates.TRACKING;
      }
    break;
  }
  return false;
}*/
  }

}
