package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.PassConstants.PassTargets;

/** An example command that uses an example subsystem. */
public class PassCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})  

  private PassTargets m_PassTargets;
  public PassCommand(PassTargets target) {
    m_PassTargets=target;
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    switch(m_PassTargets){
      case LEFT:
       //Set Target to LEFT
        break;
      case RIGHT:
      //Set Target to Right 
        break; 
      case MIDDLE:
        break;
      default:
       //default to left
       break;

    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

