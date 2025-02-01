package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DunkinDonutConstants;
import frc.robot.Constants;
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
  public void end(boolean interrupted) {
    m_dunkin.stop_rotatemotor();
    m_dunkin.resetRotateRelative();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    boolean returnValue=false;

    if(m_dunkin.getAbsoluteEncoderPos()>DunkinDonutConstants.rotateHomePos){
      m_dunkin.run_rotatemotor(DunkinDonutConstants.homeSpeed);
    }else{
      m_dunkin.run_rotatemotor(-DunkinDonutConstants.homeSpeed);
    }

    if(Math.abs(m_dunkin.getAbsoluteEncoderPos())>DunkinDonutConstants.rotateHomePos-DunkinDonutConstants.homePosDeadband&&
      Math.abs(m_dunkin.getAbsoluteEncoderPos())<DunkinDonutConstants.rotateHomePos+DunkinDonutConstants.homePosDeadband){
        returnValue=true;
    }
    return returnValue;
  }
}
