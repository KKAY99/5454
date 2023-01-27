package frc.robot.commands;

import frc.robot.subsystems.DownLiftSubSystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * An example command that uses an example subsystem.
 */
public class DownLiftCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final DownLiftSubSystem m_subsystem;
  private double m_speed;
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public DownLiftCommand(DownLiftSubSystem subsystem,double speed) {
    m_subsystem = subsystem;
    m_speed=speed;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println("Executing DownLift");
    m_subsystem.setSpeed(m_speed);
  }

 // Called once the command ends or is interrupted.
 @Override
 public void end(final boolean interrupted) {
   m_subsystem.setSpeed(0);
 }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
