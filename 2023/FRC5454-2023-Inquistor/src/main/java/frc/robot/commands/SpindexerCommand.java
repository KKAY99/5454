package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SpindexerSubsystem;

public class SpindexerCommand extends CommandBase{

private final SpindexerSubsystem m_SpindexerSubsystem;
private final double m_speed;
    public SpindexerCommand(SpindexerSubsystem spindexer,double speed){;
      m_SpindexerSubsystem = spindexer;
      m_speed=speed;
    }
    
      // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_SpindexerSubsystem.run(m_speed);
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_SpindexerSubsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
