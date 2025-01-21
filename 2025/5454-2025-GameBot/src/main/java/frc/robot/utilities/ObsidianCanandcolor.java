package frc.robot.utilities;

import com.reduxrobotics.sensors.canandcolor.Canandcolor;
import com.reduxrobotics.sensors.canandcolor.CanandcolorSettings;
import com.reduxrobotics.sensors.canandcolor.DigoutChannel;
import com.reduxrobotics.sensors.canandcolor.DigoutFrameTrigger;

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


    }
 
