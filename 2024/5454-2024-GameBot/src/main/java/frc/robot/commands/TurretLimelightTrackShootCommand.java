package frc.robot.commands;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.Swerve;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.utilities.Limelight;
import frc.robot.subsystems.TurretSubsystem;

public class TurretLimelightTrackShootCommand extends Command{
  private Limelight m_limeLight;
  private TurretSubsystem m_turret;
  private ShooterSubsystem m_shooter;
  private IntakeSubsystem m_intake;

  private boolean m_runLeft=true;
  private boolean m_runRight=false;

  private double kTimeToRunShooter=ShooterConstants.timeToRunShooter;
  private double kTimeToRunIntake=Constants.IntakeConstants.timeToRunIntake;
  private double m_currentTime;

  private enum TrackingStates{
    TRACKING,SHOOT,SEARCHING
  }

  private TrackingStates m_state=TrackingStates.SEARCHING;

  public TurretLimelightTrackShootCommand(Limelight limeLight,TurretSubsystem turret,ShooterSubsystem shooter,IntakeSubsystem intake){
    m_limeLight=limeLight;
    m_turret=turret;
    m_shooter=shooter;
    m_intake=intake;

    addRequirements(m_intake);
    addRequirements(m_turret);
    addRequirements(m_shooter);
  }

  @Override
  public void end(boolean interrupted){
    m_intake.stopIntake();
    m_turret.stop();
    m_shooter.StopShootingMotors();
    m_turret.ResetPIDReference();
  }

  @Override
  public boolean isFinished(){
    boolean returnValue=false;

    double speed=0;
    double multiplier=m_limeLight.GetDistanceMultipler();

    switch(m_state){
    case TRACKING:
      if(m_limeLight.isTargetAvailible()){
        if(Math.abs(m_limeLight.getXRaw())<Constants.LimeLightValues.limeLightDeadBand*multiplier){
          speed=0;
        }else if(Math.abs(m_limeLight.getXRaw())<Constants.LimeLightValues.closeXCheck*multiplier){
          speed=Constants.LimeLightValues.limeLightTrackSpeed1;
        }else if(Math.abs(m_limeLight.getXRaw())<Constants.LimeLightValues.medXCheck*multiplier){
          speed=Constants.LimeLightValues.limeLightTrackSpeed2;
        }else if(Math.abs(m_limeLight.getXRaw())<Constants.LimeLightValues.farXCheck*multiplier){
          speed=Constants.LimeLightValues.limeLightTrackSpeed3;
        }else{
          speed=Constants.LimeLightValues.limeLightTrackSpeed4;
        }

        if(m_limeLight.getXRaw()<0){
          m_turret.RunCheckLimits(speed);
        }else if(m_limeLight.getXRaw()>0){
          m_turret.RunCheckLimits(-speed);
        }else{
          m_state=TrackingStates.SHOOT;
          m_turret.stop();
          m_currentTime=Timer.getFPGATimestamp();
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
    case SHOOT:
      if(m_currentTime+kTimeToRunIntake<Timer.getFPGATimestamp()){
        m_intake.stopIntake();
      }else{
        m_intake.runIntake(Constants.IntakeConstants.autoIntakeSpeed);
      }

      if(m_currentTime+kTimeToRunShooter<Timer.getFPGATimestamp()){
        m_shooter.StopShootingMotors();
        returnValue=true;
        m_shooter.ShotTaken();
      }else{
        m_shooter.RunShootingMotors(Constants.ShooterConstants.autoShooterSpeed);
      }
    break;
  }
  return returnValue;
}

}
