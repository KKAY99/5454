// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import frc.robot.common.control.PidController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.classes.Limelight;
import frc.robot.subsystems.DrivetrainSubsystem;

public class PipelineSwapCommand extends CommandBase {
  private Limelight m_limelight;
  private int m_pipeline;
  private double m_targetHeight;
  private boolean m_done=false;
  private DrivetrainSubsystem m_drive;
  private PidController m_pid;

  /** Creates a new PipelineSwap. */
  public PipelineSwapCommand(Limelight limelight,DrivetrainSubsystem drive, int pipeline, double targetHeight) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_limelight = limelight;
    m_pipeline = pipeline;
    m_targetHeight=targetHeight;
    m_drive=drive;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println("Switching Pipeline - " + m_pipeline);
    m_limelight.setPipeline(m_pipeline);
    m_limelight.setTargetHeight(m_targetHeight);
    m_done=true;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(m_limelight.isTargetAvailible()){
      if(Math.abs(m_limelight.getXRaw())<0.15){
        return true; 
      }else{
        if(m_limelight.getXRaw()>0){
          System.out.println("move right");
          m_drive.move(270 ,0,.05,1,true);
        }else{
          System.out.println("move left");
          m_drive.move(90 ,0,.05,1,true);
        }
        return false;

      }


    }else{
      return m_done; // done after first execution
    }
    
  }
}
