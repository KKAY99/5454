package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.*;
import frc.robot.Constants;
/** An example command that uses an example subsystem. */
public class AutoMoveCommand extends CommandBase {
  //private final DrivetrainSubsystem m_drive;
  private final SwerveDriveGB m_drive;
  private final double m_direction;
  private final double m_distance;
  private final double m_rcw;
  private final double m_time;
  private boolean m_isFinished=false;

  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})    
  public AutoMoveCommand(SwerveDriveGB subsystem,double direction,double distance) {
    m_drive=subsystem;
    m_direction=direction;
    m_distance=distance;
    m_rcw=0; // default value - not passed in
    m_time=1.5;
    addRequirements(subsystem);
  }
  public AutoMoveCommand(SwerveDriveGB subsystem,double direction,double rcw,double distance) {
    m_drive=subsystem;
    m_direction=direction;
    m_distance=distance;
    m_rcw=rcw;
    m_time=1.5;
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_isFinished=false;
    m_drive.move(m_direction ,m_rcw,Constants.AutoModes.MoveSpeed,m_distance,m_time,true);
    m_isFinished=true;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_isFinished;
  }
}

