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
import frc.robot.classes.PhotonVision;
import org.photonvision.targeting.PhotonTrackedTarget;


public class PhotonAlign extends CommandBase {
  private Limelight m_limelight;
  private PhotonVision m_photonvision;
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
  public PhotonAlign(DrivetrainSubsystem drive, PhotonVision photonvision) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drive=drive;
    m_photonvision = photonvision;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

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
      if(m_photonvision.CanSeeAnyTarget()){
      // int id = m_photonvision.GetBestMatchAprilTagID();
      //  double measurement = m_photonvision.GetX(id);
      int id = 0;
      double measurement = m_photonvision.GetX(id);
        double filteredMeasurement = m_filter.calculate(measurement);
        if(Math.abs(filteredMeasurement)<0.8){
          System.out.println("stopping");
            m_drive.stop();
          return true; 
        }else{
          if(filteredMeasurement>0){
            System.out.println("move right");
            double pidOutput = m_pidRight.calculate(filteredMeasurement);
            pidOutput=Math.min(Math.max(pidOutput,-0.10),.10);
            System.out.println("Aligning - X is " + m_photonvision.GetX(id) + " filtered is " + filteredMeasurement + " pidOutput is " + pidOutput);
            m_drive.move(270 ,0,pidOutput,0.1,true);
          }else{
            System.out.println("move left");
            double pidOutput = m_pidLeft.calculate(filteredMeasurement);
            pidOutput=Math.min(Math.max(pidOutput,-0.10),.10);
           
            System.out.println("Aligning - X is " + m_photonvision.GetX(id) + " filtered is " + filteredMeasurement + " pidOutput is " + pidOutput);
            m_drive.move(90 ,0,pidOutput,0.1,true);
          }
      
          double distanceErrorMeasurement = m_photonvision.GetTargetDistance(id);
          double filteredDistanceErrorMeasurement = m_filter.calculate(distanceErrorMeasurement)-1;
          if(Math.abs(filteredDistanceErrorMeasurement)<0.8 || distanceErrorMeasurement < 0){
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
          
        }
        return false;
      }
      
    }else{

        return m_done; // done after first execution
    }   
  }
}


