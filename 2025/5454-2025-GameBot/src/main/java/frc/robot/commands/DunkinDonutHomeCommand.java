package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DunkinDonutSubsystem;

public class DunkinDonutHomeCommand extends Command {
  private DunkinDonutSubsystem m_dunkin;

  public DunkinDonutHomeCommand(DunkinDonutSubsystem dunkin) {
    m_dunkin=dunkin;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
