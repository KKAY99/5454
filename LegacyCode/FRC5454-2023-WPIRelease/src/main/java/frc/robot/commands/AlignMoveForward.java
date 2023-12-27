// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.classes.Limelight;
import frc.robot.subsystems.DrivetrainSubsystem;

public class AlignMoveForward extends CommandBase {
  private Limelight m_limelight;
  private int m_pipeline;
  private double m_targetHeight;
  private boolean m_done=false;
  private DrivetrainSubsystem m_drive;
  private PIDController m_pid = new PIDController(3.0,0.0,0.1, 0.05);
  private double m_speed = 0.05;

  /** Creates a new PipelineSwap. */
  public AlignMoveForward(Limelight limelight,DrivetrainSubsystem drive,int pipeline, double targetHeight) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_limelight = limelight;
    m_targetHeight=targetHeight;
    m_pipeline = pipeline;
    m_drive=drive;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(m_limelight.getPipeline()!=m_pipeline){
      System.out.println("Switching Pipeline - " + m_pipeline + " from " + m_limelight.getPipeline());
        m_limelight.setPipeline(m_pipeline);
        m_limelight.setTargetHeight(m_targetHeight);
      }
      m_done=true;
    }
  

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    
    if(m_limelight.getDistance() <= 13.5){
      return true;
    }else{
      m_drive.move(0,0,0.4,1,false);
      System.out.println(m_limelight.getDistance());
    }
    
    return false;
  }
}