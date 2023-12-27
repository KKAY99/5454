// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.classes.Limelight;
import frc.robot.subsystems.DrivetrainSubsystem;
import edu.wpi.first.math.filter.MedianFilter;
import frc.robot.Constants;
import frc.robot.Constants.PIDSteering;


public class PipelineSwapCommand extends CommandBase {
  private Limelight m_limelight;
  private int m_pipeline;
  private double m_targetHeight;
  private boolean m_done=false;
  private DrivetrainSubsystem m_drive;
 // Limelight sensors tend to be quite noisy and susceptible to sudden outliers,
  // so measurements are filtered with a 5-sample median filter
  private final MedianFilter m_filter = new MedianFilter(3);
  private PIDController m_pidRight = new PIDController(Constants.PIDSteering.rightKP,PIDSteering.rightKI,PIDSteering.rightKD);
  private PIDController m_pidLeft = new PIDController(Constants.PIDSteering.leftKP,PIDSteering.leftKI,PIDSteering.leftKD);
  private PIDController m_pidForward = new PIDController(Constants.PIDSteering.forwardKP,PIDSteering.forwardKI,PIDSteering.forwardKD);
  private PIDController m_pidBackward = new PIDController(Constants.PIDSteering.backwardKP,PIDSteering.backwardKI,PIDSteering.backwardKD);
  
  private double m_speed = 0.05;

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
      double measurement = m_limelight.getXRaw();
      double filteredMeasurement = m_filter.calculate(measurement);
      if(m_limelight.isTargetAvailible()){
        if(Math.abs(filteredMeasurement)<0.8){
          System.out.println("stopping");
            m_drive.stop();
          return true; 
        }else{
          if(filteredMeasurement>0){
            System.out.println("move right");
            double pidOutput = m_pidRight.calculate(filteredMeasurement);
            pidOutput=Math.min(Math.max(pidOutput,-0.10),.10);
            System.out.println("Aligning - X is " + m_limelight.getXRaw() + " filtered is " + filteredMeasurement + " pidOutput is " + pidOutput);
            m_drive.move(270 ,0,pidOutput,1,true);
          }else{
            System.out.println("move left");
            double pidOutput = m_pidLeft.calculate(filteredMeasurement);
            pidOutput=Math.min(Math.max(pidOutput,-0.10),.10);
           
            System.out.println("Aligning - X is " + m_limelight.getXRaw() + " filtered is " + filteredMeasurement + " pidOutput is " + pidOutput);
            m_drive.move(90 ,0,pidOutput,1,true);
          }
            
      /*double distanceErrorMeasurement = m_limelight.getYRaw();
      double filteredDistanceErrorMeasurement = m_filter.calculate(distanceErrorMeasurement)+20;
        if(m_limelight.isTargetAvailible()){
          if(Math.abs(filteredDistanceErrorMeasurement)<0.1){
            System.out.println("stopping");
            m_drive.stop();
            return true; 
          }else{
            if(filteredDistanceErrorMeasurement>0){
            System.out.println("move Forward");
            double pidOutputDistance = m_pidForward.calculate(filteredDistanceErrorMeasurement);
            pidOutputDistance=Math.min(Math.max(pidOutputDistance,-0.10),.10);
            System.out.println("Aligning - Y is " + distanceErrorMeasurement + " filtered is " + filteredDistanceErrorMeasurement + " pidOutput is " + pidOutputDistance);
            m_drive.move(0 ,0,pidOutputDistance,1,true);
          }else{
            System.out.println("move Back");
            double pidOutputDistance = m_pidBackward.calculate(filteredDistanceErrorMeasurement);
            pidOutputDistance=Math.min(Math.max(pidOutputDistance,-0.10),.10);
              
            System.out.println("Aligning - Y is " + distanceErrorMeasurement + " filtered is " + filteredDistanceErrorMeasurement + " pidOutput is " + pidOutputDistance);
            m_drive.move(180 ,0,pidOutputDistance,1,true);
                 

          }
          return false;
        
        }*/
          return false;
        }
      }else{

        return m_done; // done after first execution
      }   

    } 
  } 


