package frc.robot.commands;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.PIDSteering;
import frc.robot.classes.Limelight;
import frc.robot.subsystems.DrivetrainSubsystem;
import edu.wpi.first.math.filter.MedianFilter;



public class zAutoTargetandMove extends CommandBase{
private int m_gridChoice=0;
private Limelight m_limeLight;
private int m_pipeline=0;
private double m_targetHeight;
private DrivetrainSubsystem m_drive;
private final MedianFilter m_filter = new MedianFilter(5);
private PIDController m_pidRight = new PIDController(Constants.PIDSteering.rightKP,PIDSteering.rightKI,PIDSteering.rightKD);
private PIDController m_pidLeft = new PIDController(Constants.PIDSteering.leftKP,PIDSteering.leftKI,PIDSteering.leftKD);



public zAutoTargetandMove(Limelight limelight,DrivetrainSubsystem drive,int gridChoice){
        m_gridChoice=gridChoice;
        m_limeLight=limelight;
        m_drive=drive;
        //GRID Choice are 1 to 9 for the three rows in a grid 
        //GRID 10 (top cone any column) 11 (middle cone any column) 12 (botton cone any column)
        //GRID 13 (top cube  column) 14 (middle cube any column), 15 (bottom cube any column)
        switch (gridChoice)
        {
        //     //fall thorugh cases for same pipelines and heights
        //     case Constants.ChargedUp.GridPosUpperLeft:
        //     case Constants.ChargedUp.GridPosUpperRight:
        //     case Constants.ChargedUp.GridPosUpperConeAny:
        //         m_pipeline=Constants.VisionPipelines.TopTape;
        //         m_targetHeight=Constants.ChargedUp.targetHeightHighTape;
        //         break;
        //     case Constants.ChargedUp.GridPosMiddleLeft:
        //     case Constants.ChargedUp.GridPosMiddleRight:
        //     case Constants.ChargedUp.GridPosMiddleConeAny:
        //     case Constants.ChargedUp.GridPosBottomLeft:
        //     case Constants.ChargedUp.GridPosBottomRight:
        //     case Constants.ChargedUp.GridPosBottomConeAny:
        //         m_pipeline=Constants.VisionPipelines.BottomTape;
        //         m_targetHeight=Constants.ChargedUp.targetHeighMLowTape;
        //         break;
        //     default:
        //         // Assume AprilTag if not a Cone Position
        //         m_pipeline=Constants.VisionPipelines.AprilTag;
        //         m_targetHeight=Constants.ChargedUp.targetHeightAprilTag;
            
        // }
        }
    }
    @Override
    public void initialize() {

    }
  
     @Override
    public void execute() {
        if(m_limeLight.getPipeline()!=m_pipeline){
            System.out.println("Switching Pipeline - " + m_pipeline + " from " + m_limeLight.getPipeline());
              m_limeLight.setPipeline(m_pipeline);
              m_limeLight.setTargetHeight(m_targetHeight);
        }
    }
    
    
  
    @Override
    public void end(boolean interrupted) {
    
    }
  
    @Override
    public boolean isFinished() {
        boolean returnValue=false;
        if(m_limeLight.isTargetAvailible()){
          if(Math.abs(m_limeLight.getXRaw())<0.15){ 
            System.out.println("stopping on target");
            m_drive.stop();
            returnValue=true;;
          }else {
            
            double measurement = m_limeLight.getXRaw();
            double filteredMeasurement = m_filter.calculate(measurement);
            if(filteredMeasurement>0){
                  double pidOutput = m_pidRight.calculate(filteredMeasurement);
                  System.out.println("Aligning - X is " + m_limeLight.getXRaw() + " filtered is " + filteredMeasurement + " pidOutput is " + pidOutput);
                  m_drive.move(90 ,0, pidOutput ,0.1,false);
                }
                  else{
                  double pidOutput = m_pidLeft.calculate(filteredMeasurement);
                  System.out.println("Aligning - X is " + m_limeLight.getXRaw() + " filtered is " + filteredMeasurement + " pidOutput is " + pidOutput);
                  m_drive.move(270 ,0, pidOutput ,0.1,false);
                }
              }         
        }
        return returnValue;
    
          }
    
}
