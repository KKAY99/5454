package frc.robot.classes;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.jni.*;
public class WPIAprilTag {
    public WPIAprilTag()
    {
        CameraServer.startAutomaticCapture();
        
    }
    public boolean seeAprilTag(){
        AprilTag aprilTag = new AprilTag(0, null)
        april
    }
}
