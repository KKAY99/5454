package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DunkinDonutSubsystem;
import frc.robot.Constants.DunkinDonutConstants;

public class DunkinDonutPosCommand extends Command {
  private DunkinDonutSubsystem m_dunkin;
  private double m_pos;

  public DunkinDonutPosCommand(DunkinDonutSubsystem dunkin, double pos) {
    m_dunkin=dunkin;
    m_pos = pos;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_dunkin.set_referance(m_pos);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
     m_dunkin.reset_referance();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    boolean returnValue=false;

    if (m_dunkin.get_rotatemotorpos()>m_pos-DunkinDonutConstants.posDeadband&&m_dunkin.get_rotatemotorpos()<m_pos+DunkinDonutConstants.posDeadband){
       returnValue=true;
    }
     
    return returnValue;
  }
}
