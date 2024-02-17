package frc.robot.commands;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.utilities.Limelight;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.Swerve;

public class StaticLimelightTrackShootCommand extends Command{
  private Limelight m_limeLight;
  private TurretSubsystem m_turret;
  private Swerve m_swerve;
  private ShooterSubsystem m_shooter;
  private IntakeSubsystem m_intake;

  private double kTimeToRunShooter=ShooterConstants.timeToRunShooter;
  private double kTimeToRunIntake=Constants.IntakeConstants.timeToRunIntake;
  private double m_currentTime;

  private enum TrackingStates{
    TRACKING,SHOOT
  }

  private TrackingStates m_state=TrackingStates.TRACKING;

  public StaticLimelightTrackShootCommand(Limelight limeLight,TurretSubsystem turret,Swerve swerve,ShooterSubsystem shooter,IntakeSubsystem intake){
    m_limeLight=limeLight;
    m_turret=turret;
    m_swerve=swerve;
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
      m_turret.TurretSetReference(Constants.TurretConstants.turretStraightPos);

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
          m_swerve.drive(new Translation2d(0,0),speed,true,true);
        }else if(m_limeLight.getXRaw()>0){
          m_swerve.drive(new Translation2d(0,0),-speed,true,true);
        }else{
          m_state=TrackingStates.SHOOT;
          m_swerve.drive(new Translation2d(0,0),0,true,true);
          m_currentTime=Timer.getFPGATimestamp();
        }
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
