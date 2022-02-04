package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.*;
import frc.robot.Constants;
/** An example command that uses an example subsystem. */
public class AutoMoveForwardCommand extends CommandBase {
  private DriveSubsystem m_drive;
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})    
  public AutoMoveForwardCommand(DriveSubsystem subsystem) {
    m_drive=subsystem;
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_drive.move(0,Constants.AutoModes.MoveSpeed,Constants.AutoModes.LeaveTarmacDistance,true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

