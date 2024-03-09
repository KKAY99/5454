package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.NoteFlipperSubsystem;

public class NoteFlipCommand extends Command {
  private NoteFlipperSubsystem m_flip;

  private double m_speed;

  public NoteFlipCommand(NoteFlipperSubsystem flip,double speed){
    m_flip=flip;
    m_speed=speed;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute(){
    m_flip.run(m_speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted){
    m_flip.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
