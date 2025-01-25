package frc.robot.utilities;

import com.reduxrobotics.sensors.canandcolor.Canandcolor;
import com.reduxrobotics.sensors.canandcolor.CanandcolorSettings;
import com.reduxrobotics.sensors.canandcolor.DigoutChannel;
import com.reduxrobotics.sensors.canandcolor.DigoutFrameTrigger;
import com.reduxrobotics.sensors.canandcolor.ProximityPeriod;

public class ObsidianCanandcolor {

    private Canandcolor m_sensor;
    private CanandcolorSettings m_settings;
    private static final double kDefaultStatusFramePeriod=1.0;
    private static final double kDefaultProximityFramePeriod=0.01;
    private static final double kDigoutFramePeriod=0.100;
    public ObsidianCanandcolor(int canID){
        m_sensor= new Canandcolor(canID);
        m_settings=m_sensor.getSettings();  // This is a Blocking Call but acceptable during robot init when all objects are created
        m_settings.setProximityFramePeriod(kDefaultProximityFramePeriod);
        m_settings.setStatusFramePeriod(kDefaultStatusFramePeriod);
        m_settings.setDigoutFramePeriod(kDigoutFramePeriod);
        //Tells the device to early transmit a digital output packet if the status changes at all
         m_settings.setDigoutFrameTrigger(DigoutChannel.Index.kDigout1, DigoutFrameTrigger.kRisingAndFalling);
    }   
    public void setLEDLight(double brightness){
        m_sensor.setLampLEDBrightness(brightness);
    }
    public void setProximityFramePeriod(double period){
        m_settings.setProximityFramePeriod(period);
    }
    public double getProximity(){
        //Gets the currently sensed proximity normalized between [0..1] inclusive.
        //The value decreases as an object gets closer to the sensor.        
        //Note that proximity is not given a unit as different materials and sensor
        //configurations can greatly vary how the proximity value translates to actual 
        //real-world units. It is generally presumed that users will have to finetune 
        //specific thresholds for applications anyway and units may not be meaningful or accurate.
      return m_sensor.getProximity();
      
    }
    public void setProximityIntegrationPeriod(int sec){
        switch(sec){
            case 5:
                m_settings.setProximityIntegrationPeriod(ProximityPeriod.k5ms); 
                break;
            case 10:
                m_settings.setProximityIntegrationPeriod(ProximityPeriod.k10ms); 
                break;          
            case 20:
                m_settings.setProximityIntegrationPeriod(ProximityPeriod.k20ms); 
                break;
            case 40:
                m_settings.setProximityIntegrationPeriod(ProximityPeriod.k40ms); 
                break;
            default:
                m_settings.setProximityIntegrationPeriod(ProximityPeriod.k20ms); 
                break;
            }
     
    }
    public String getColorHexString(){
       // Logger.recordOutput("CanandColor", null);
        return m_sensor.getColor().toWpilibColor().toHexString();
    }
    
}
 
