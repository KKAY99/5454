package frc.robot.commands;
import frc.robot.classes.Limelight;
import frc.robot.subsystems.DrivetrainSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.controller.PIDController;
import frc.robot.Constants.PIDSteering;

public class zAutoTargetToColumnCommand extends CommandBase{
    private Limelight m_limeLight;
    private DrivetrainSubsystem m_drive;
    private int m_pipeline;
    private double m_targetHeight;
    private double m_targetColumnDistance;
    private final MedianFilter m_filter = new MedianFilter(5);
    private PIDController m_pidRight = new PIDController(Constants.PIDSteering.rightKP,PIDSteering.rightKI,PIDSteering.rightKD);
    private PIDController m_pidLeft = new PIDController(Constants.PIDSteering.leftKP,PIDSteering.leftKI,PIDSteering.leftKD);
        
    public zAutoTargetToColumnCommand(Limelight limelight,DrivetrainSubsystem drive,int gridChoice){
        m_limeLight = limelight;
        m_drive = drive;
        switch(gridChoice)
        {
            case Constants.ChargedUp.GridPosUpperLeft:
            case Constants.ChargedUp.GridPosMiddleLeft:
            case Constants.ChargedUp.GridPosBottomLeft:
            m_targetColumnDistance = Constants.ChargedUp.leftTargetPositionX;
            break;
            case Constants.ChargedUp.GridPosUpperRight:
            case Constants.ChargedUp.GridPosMiddleRight:
            case Constants.ChargedUp.GridPosBottomRight:
            m_targetColumnDistance = Constants.ChargedUp.rightTargetPositionX;
            break;
            default:
                // Assume AprilTag if not a Cone Position
                m_targetColumnDistance = Constants.ChargedUp.middleTargetPositionX;
            

        }
        switch(gridChoice)
        {
            //fall thorugh cases for same pipelines and heights
            case Constants.ChargedUp.GridPosUpperLeft:
            case Constants.ChargedUp.GridPosUpperRight:
            case Constants.ChargedUp.GridPosUpperConeAny:
                m_pipeline=Constants.VisionPipelines.TopTape;
                m_targetHeight=Constants.ChargedUp.targetHeightHighTape;
                break;
            case Constants.ChargedUp.GridPosMiddleLeft:
            case Constants.ChargedUp.GridPosMiddleRight:
            case Constants.ChargedUp.GridPosMiddleConeAny:
            case Constants.ChargedUp.GridPosBottomLeft:
            case Constants.ChargedUp.GridPosBottomRight:
            case Constants.ChargedUp.GridPosBottomConeAny:
                m_pipeline=Constants.VisionPipelines.BottomTape;
                m_targetHeight=Constants.ChargedUp.targetHeighMLowTape;
                break;
            default:
                // Assume AprilTag if not a Cone Position
                m_pipeline=Constants.VisionPipelines.AprilTag;
                m_targetHeight=Constants.ChargedUp.targetHeightAprilTag;
            
        }
      
    }

    @Override
    public void execute(){
        m_limeLight.setPipeline(Constants.VisionPipelines.AprilTag);
    }

    @Override
    public boolean isFinished(){
        boolean returnValue = false;
        boolean forward = false;
        boolean back = false;
        boolean right = false;
        boolean left = false;
        int direction=0;
        double speed=0;
        int rotation=0; 
        
        if(m_limeLight.getX()>Constants.ChargedUp.AprilTagAlignmentToleranceX){
            right = true;

        }else{
            if (m_limeLight.getX()<-(0-Constants.ChargedUp.AprilTagAlignmentToleranceX)){
                left = true;
            }
        }
            
        if(m_limeLight.getArea()<(Constants.ChargedUp.distanceFromTag-Constants.ChargedUp.AprilTagAlignmentToleranceArea)){
            forward = true;
        }else{
            if(m_limeLight.getArea()>Constants.ChargedUp.distanceFromTag+Constants.ChargedUp.AprilTagAlignmentToleranceArea){
                back = true;
            }
        }
        double measurement = m_limeLight.getXRaw();
        double filteredMeasurement = m_filter.calculate(measurement);
        double pidRightOutput = m_pidRight.calculate(filteredMeasurement);
        double pidLeftOutput = m_pidLeft.calculate(filteredMeasurement);
        
        pidLeftOutput=Math.min(Math.max(pidLeftOutput,-0.10),.10);
        pidRightOutput=Math.min(Math.max(pidRightOutput,-0.10),.10);
     
        //determine direction, rotation and speed to move
        if(m_limeLight.isTargetAvailible()){
            if(forward||back||right||left){
                    if(left){
                        speed=pidLeftOutput;
                        if(forward){
                           direction=45;                                    
                        }else if(back){
                            direction=135;
                        }else{
                           direction=90;                          
                        }
                    }else{
                        if(right){
                            speed=pidRightOutput;
                            if(forward){
                               direction=315;                                           
                            }else if(back){
                                direction=235;                           
                               }else{
                                direction=270;                                               
                               }
        
                        }else{
                            speed=0.1; // TODO Determine optimal front / back speed
                            if(forward){
                               direction=0;                             
                            }else{
                                direction=180;
                            }
                        }
        
                    }
                    //System.out.println("xPos " + filteredMeasurement+ " direction " + direction + " -- speed " +speed);
                    m_drive.movenodistance(direction, rotation, speed);
                    returnValue = false;
                }else{
                    //System.out.println("Is Not moving");
                    m_drive.stop();
                    returnValue = true; 
                    
                }
        }else{
            m_drive.stop();
            returnValue = true; 
        }
        return returnValue;
    }
}
 