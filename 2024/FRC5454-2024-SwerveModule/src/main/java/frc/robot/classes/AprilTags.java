package frc.robot.classes;
import frc.robot.Constants;

public class AprilTags {
    private Limelight m_limeLight;

    public  AprilTags(Limelight limelight){
        m_limeLight=limelight;
    }

    private boolean limelightFilteredPipeline(int pipeline){
        m_limeLight.setPipeline(pipeline);
        return m_limeLight.isOnTargetX();
    }

    public int getAutoTurn(){

      //  if (limelightFilteredPipeline(Constants.VisionPipelines.AprilTagID1) || 
      //      limelightFilteredPipeline(Constants.VisionPipelines.AprilTagID6)){
//
 //               return 90;
 //       }else {
 //               return 270;
 //       }
    return 90; // put in to compile
    }
}
