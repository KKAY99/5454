package frc.robot.utilities;
import edu.wpi.first.wpilibj.AnalogInput;

public class AnalogAutoDirectFB6DN0E {
    private AnalogInput m_sensor;
    private final static double kBeamThreshold=800;
   
    public AnalogAutoDirectFB6DN0E(int port){
        try{
        m_sensor=new AnalogInput(port);
        }
        catch(Exception e){
            System.out.println(e.getMessage());
        }
    }
    public boolean isBeamBroken(){
        if(m_sensor!=null){
            if (m_sensor.getValue()<kBeamThreshold){
                return true;
            } else {
                return false;
            }
        }else{
            //null sensor
            return false;
        }
    }

    public int getRawValue(){
        if(m_sensor!=null){
               return m_sensor.getValue();
        } else {
            return 0;
        }
    }

}
