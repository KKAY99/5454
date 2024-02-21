package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve;

public class GyroResetCommand extends Command {
  private Swerve m_swerve;

  public GyroResetCommand(Swerve swerve){
    m_swerve=swerve;
  }

  @Override
  public void execute(){
    m_swerve.zeroGyro();
  }

  @Override
  public void end(boolean interrupted){}

  @Override
  public boolean isFinished(){
    return true;
  }
}
