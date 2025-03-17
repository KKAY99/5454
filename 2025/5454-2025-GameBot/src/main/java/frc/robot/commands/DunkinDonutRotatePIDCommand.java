package frc.robot.commands;

import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DunkinDonutSubsystem;

public class DunkinDonutRotatePIDCommand extends Command {
  private DunkinDonutSubsystem m_dunkin;
  private double m_targetPos;

  public DunkinDonutRotatePIDCommand(DunkinDonutSubsystem dunkin, double targetPos) {
    m_dunkin=dunkin;
    m_targetPos=targetPos;

    addRequirements(dunkin);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("Stowing to :" + m_targetPos);
    m_dunkin.toggleLocalPid(m_targetPos);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
   
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
