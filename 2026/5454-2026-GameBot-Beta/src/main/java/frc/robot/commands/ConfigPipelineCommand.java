package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.utilities.Limelight;

public class ConfigPipelineCommand extends Command {
  private Limelight m_limeLight;

  private double[] m_xyzOffset;
  private double m_targetFiducial;

  public ConfigPipelineCommand(Limelight limelight,double[] xyzOffset,double targetFiducial) {
    m_limeLight=limelight;
    m_xyzOffset=xyzOffset; 
    m_targetFiducial=targetFiducial;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute(){
    m_limeLight.setXYZFiducialOffset(m_xyzOffset);
    m_limeLight.setLimelightIDFilter(m_targetFiducial);
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
