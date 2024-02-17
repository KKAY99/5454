package frc.robot.commands;
import frc.robot.utilities.Limelight;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.commands.TurretLimelightTrackShootCommand;
import frc.robot.commands.StaticLimelightTrackShootCommand;

public class LimelightTargetingSelector extends Command {
  private Limelight m_turretLimelight;
  private Limelight m_staticLimelight;

  private TurretSubsystem m_turret;
  private Swerve m_swerve;
  private ShooterSubsystem m_shooter;
  private IntakeSubsystem m_intake;

  public LimelightTargetingSelector(Limelight turretLimelight,Limelight staticLimelight,TurretSubsystem turret,Swerve swerve,ShooterSubsystem shooter,IntakeSubsystem intake) {
    m_turretLimelight=turretLimelight;
    m_staticLimelight=staticLimelight;
  }

  @Override
  public void execute(){
    if(m_staticLimelight.isTargetAvailible()){
      CommandScheduler.getInstance().schedule(new StaticLimelightTrackShootCommand(m_staticLimelight,m_turret,m_swerve,m_shooter,m_intake));
    
    }else{
      CommandScheduler.getInstance().schedule(new TurretLimelightTrackShootCommand(m_turretLimelight,m_turret,m_shooter,m_intake));
    }
  }

  @Override
  public void end(boolean interrupted){}

  @Override
  public boolean isFinished() {
    return true;
  }
}
