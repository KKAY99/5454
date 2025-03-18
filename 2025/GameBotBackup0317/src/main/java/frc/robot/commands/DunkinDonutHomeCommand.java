package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DunkinDonutConstants;
import frc.robot.Constants;
import frc.robot.subsystems.DunkinDonutSubsystem;

public class DunkinDonutHomeCommand extends Command {
  private DunkinDonutSubsystem m_dunkin;

  public DunkinDonutHomeCommand(DunkinDonutSubsystem dunkin) {
    m_dunkin=dunkin;
    addRequirements(m_dunkin);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_dunkin.resetShouldRunPID();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_dunkin.stop_rotatemotor();
    m_dunkin.resetRotateRelative();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    m_dunkin.toggleLocalPid(DunkinDonutConstants.rotateHomePos);

    return true;
  }
}
