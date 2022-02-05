package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.*;
import frc.robot.Constants;
/** An example command that uses an example subsystem. */
public class AutoMoveCommand extends CommandBase {
  private final DriveSubsystem m_drive;
  private final double m_direction;
  private boolean m_isFinished=false;

  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})    
  public AutoMoveCommand(DriveSubsystem subsystem,double direction) {
    m_drive=subsystem;
    m_direction=direction;
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_isFinished=false;
    m_drive.move(m_direction ,Constants.AutoModes.MoveSpeed,Constants.AutoModes.LeaveTarmacDistance,true);
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

