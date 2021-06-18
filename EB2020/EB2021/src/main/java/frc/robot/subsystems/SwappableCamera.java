package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;

public class SwappableCamera extends SubsystemBase {
                                                                      private int m_activeCamera=0;
    private UsbCamera m_Camera1;
    private USbCamera m_Camera2; 
    /**
   * Creates a new IntakeSubsystem.
   */
  public SwappableCamera(int Cameras) {
      m_Camera1=CameraServer.getInstance().startAutomaticCapture(0);
      if(Cameras>1){
        m_Camera2=CameraServer.getInstance().startAutomaticCapture(1);
      }
    
  }

 public void StartCameras(int CameraNumber)
{
    m_USBCamera=CameraServer.getInstance().startAutomaticCapture(CameraNumber);  
    
}
public void EndCamera(int CameraNumber)
{
    m_USBCamera=CameraServer.getInstance();  
    
}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
