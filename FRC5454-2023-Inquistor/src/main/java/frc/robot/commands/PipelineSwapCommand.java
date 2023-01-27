// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.PIDSteering;
import frc.robot.classes.Limelight;
import frc.robot.subsystems.DrivetrainSubsystem;
import edu.wpi.first.math.filter.MedianFilter;

public class PipelineSwapCommand extends CommandBase {
  private Limelight m_limelight;
  private int m_pipeline;
  private double m_targetHeight;
  private boolean m_done=false;
  private DrivetrainSubsystem m_drive;
 // Limelight sensors tend to be quite noisy and susceptible to sudden outliers,
  // so measurements are filtered with a 5-sample median filter
  private final MedianFilter m_filter = new MedianFilter(5);
  private PIDController m_pid = new PIDController(3.0,0.0,0.1, 0.05);
  private PIDController m_pidRight = new PIDController(Constants.PIDSteering.rightKP,PIDSteering.KI,PIDSteering.KD);
  private PIDController m_pidLeft = new PIDController(Constants.PIDSteering.leftKP,PIDSteering.KI,PIDSteering.KD);
  
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
   if(m_limelight.getPipeline()!=m_pipeline){
    System.out.println("Switching Pipeline - " + m_pipeline + " from " + m_limelight.getPipeline());
      m_limelight.setPipeline(m_pipeline);
      m_limelight.setTargetHeight(m_targetHeight);
    }
    m_done=true;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    boolean returnValue=false;
    if(m_limelight.isTargetAvailible()){
      if(Math.abs(m_limelight.getXRaw())<0.15){ 
        System.out.println("stopping on target");
        m_drive.stop();
        returnValue=true;;
      }else {
        
        double measurement = m_limelight.getXRaw();
        double filteredMeasurement = m_filter.calculate(measurement);
        if(filteredMeasurement>0){
              double pidOutput = m_pidRight.calculate(filteredMeasurement);
              System.out.println("Aligning - X is " + m_limelight.getXRaw() + " filtered is " + filteredMeasurement + " pidOutput is " + pidOutput);
              m_drive.move(90 ,0, pidOutput ,0.1,false);
            }
              else{
              double pidOutput = m_pidLeft.calculate(filteredMeasurement);
              System.out.println("Aligning - X is " + m_limelight.getXRaw() + " filtered is " + filteredMeasurement + " pidOutput is " + pidOutput);
              m_drive.move(270 ,0, pidOutput ,0.1,false);
            }
          }         
    }
    return returnValue;

      }

    
  }

