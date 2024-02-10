package frc.robot.utilities;
import edu.wpi.first.wpilibj.AnalogInput;

public class AnalogAutoDirectFB6DN0E {
    private AnalogInput m_sensor;
    private final static double kBeamThreshold=800;
   
    public AnalogAutoDirectFB6DN0E(int port){
        m_sensor=new AnalogInput(port);
    }
    public boolean isBeamBroken(){
        if (m_sensor.getValue()<kBeamThreshold){
            return true;
        } else {
            return false;
        }
    }

    public int getRawValue(){
        return m_sensor.getValue();
    }

}
