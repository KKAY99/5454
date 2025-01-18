package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.utilities.Limelight;

public class PipelineSwapCommand extends Command {
  private Limelight m_Limelight;

  private int m_pipeline;

  public PipelineSwapCommand(Limelight limelight, int pipeline) {
    m_Limelight=limelight;
    m_pipeline=pipeline; 
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute(){
    m_Limelight.setPipeline(m_pipeline);
    System.out.println("Set pipeline");
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted){}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
