

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;
public class ShooterInclineCommand extends Command {
  private ShooterSubsystem m_shooter;
  private double m_position;

  public ShooterInclineCommand(ShooterSubsystem shooter,double position) {
    m_shooter = shooter;
    m_position = position;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //m_shooter.ResetReference();
    m_shooter.runShooterIncline(m_position);
    //m_shooter.SetReference(m_position);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooter.stopShooterIncline();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
