package frc.robot.classes;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.util.Color;

public class RevColorSensor {
     /**
   * Change the I2C port below to match the connection of your color sensor
   */
  private I2C.Port i2cPort;

  /**
   * A Rev Color Sensor V3 object is constructed with an I2C port as a 
   * parameter. The device will be automatically initialized with default 
   * parameters.
   */
  private ColorSensorV3 m_colorSensor;

  /**
   * A Rev Color Match object is used to register and detect known colors. This can 
   * be calibrated ahead of time or during operation.
   * 
   * This object uses a simple euclidian distance to estimate the closest match
   * with given confidence range.
   */
  private ColorMatch m_colorMatcher=new ColorMatch();
  private final Color kBlueTarget = new Color(0.143, 0.427, 0.429);
  private final Color kGreenTarget = new Color(0.197, 0.561, 0.240);
  private final Color kRedTarget = new Color(0.561, 0.232, 0.114);
  private final Color kYellowTarget = new Color(0.361, 0.524, 0.113);

  public RevColorSensor(int port){
      i2cPort= I2C.Port.kOnboard;
      m_colorSensor= new ColorSensorV3(i2cPort);
    
  }
  
  public String getColorString(){
    String colorString="";  
    Color detectedColor=m_colorSensor.getColor();
    ColorMatchResult match = m_colorMatcher.matchClosestColor(detectedColor);

        if (match.color == kBlueTarget) {
          colorString = "Blue";
        } else if (match.color == kRedTarget) {
          colorString = "Red";
        } else if (match.color == kGreenTarget) {
          colorString = "Green";
        } else if (match.color == kYellowTarget) {
          colorString = "Yellow";
        } else {
          colorString = "Unknown";
        }
  
      return colorString;
  }

}
