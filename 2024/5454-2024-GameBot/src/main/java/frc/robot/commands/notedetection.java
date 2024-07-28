package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.utilities.photonvision;

public class notedetection extends Command {
  public photonvision m_photoncamera;
  
  public notedetection(photonvision photoncamera) {
    m_photoncamera = photoncamera;
    // Use addRequirements() here to declare subsystem dependencies.
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
