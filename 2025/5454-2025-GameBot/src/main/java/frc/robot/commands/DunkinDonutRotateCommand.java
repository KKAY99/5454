package frc.robot.commands;

import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DunkinDonutSubsystem;

public class DunkinDonutRotateCommand extends Command {
  private DunkinDonutSubsystem m_dunkin;
  private DoubleSupplier m_speed;

  public DunkinDonutRotateCommand(DunkinDonutSubsystem dunkin, DoubleSupplier speed) {
    m_dunkin=dunkin;
    m_speed = speed;

    addRequirements(dunkin);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_dunkin.reset_referance();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //m_dunkin.run_rotatemotor(m_speed.getAsDouble());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_dunkin.stop_rotatemotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    m_dunkin.runRotateWithLimits(-m_speed.getAsDouble());
    return false;
  }
}
