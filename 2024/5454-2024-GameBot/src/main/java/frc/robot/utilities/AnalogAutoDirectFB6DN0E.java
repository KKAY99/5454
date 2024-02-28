package frc.robot.utilities;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
public class AnalogAutoDirectFB6DN0E {
   // private AnalogInput m_sensor;
   private DigitalInput m_sensor;
   private final static double kBeamThreshold=800;
   
    public AnalogAutoDirectFB6DN0E(int port){
        try{
        m_sensor=new DigitalInput(port);
        }
        catch(Exception e){
            System.out.println(e.getMessage());
        }
    }
    public boolean isBeamBroken(){
        boolean returnValue=false;
        if(m_sensor!=null){
             returnValue= m_sensor.get();
        }
        
        return returnValue;
    }

    public int getRawValue(){
       /*  if(m_sensor!=null){
               return m_sensor.getValue();
        } else {
            return 0;
        }*/
        return 99;

    }

}
