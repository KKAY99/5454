package frc.robot.commands;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.math.geometry.Translation2d;
/** An example command that uses an example subsystem. */
public class AutoDelayedTimedMove extends Command {
private double m_startTime;
private static final double kDelayTime=1;
private static final double kDriveTime=1.5;
private static final double kDriveSpeed=-0.5;
private Swerve m_swerve;

  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})  

  public AutoDelayedTimedMove(Swerve swerve) {
    m_swerve=swerve;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_startTime=Timer.getFPGATimestamp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_swerve.drive(new Translation2d(0,0),0,false,false); // stop drive
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    double m_currentTime=Timer.getFPGATimestamp();
    boolean returnValue=false;
    if(m_currentTime<m_startTime+kDelayTime){
      //do nothing
    } else if (m_currentTime<m_startTime+kDelayTime+kDriveTime){
        m_swerve.drive(new Translation2d(0,kDriveSpeed),0,false,false);
    }else {
      returnValue=true;
    }
    return returnValue;
  }
}

