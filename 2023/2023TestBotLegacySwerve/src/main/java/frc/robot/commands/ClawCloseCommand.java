
package frc.robot.commands;

import frc.robot.subsystems.ClawSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class ClawCloseCommand extends CommandBase {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  private final ClawSubsystem m_ClawSubsystem;
  

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ClawCloseCommand(ClawSubsystem claw) {
    m_ClawSubsystem = claw;
  // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_ClawSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_ClawSubsystem.CloseClaw();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
      }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
