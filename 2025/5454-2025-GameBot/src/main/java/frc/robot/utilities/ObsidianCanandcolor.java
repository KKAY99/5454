package frc.robot.utilities;

import org.littletonrobotics.junction.Logger;
import com.reduxrobotics.sensors.canandcolor.*;

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
    public void setNeroZeroMode(){
        //Sets all frame periods to 0.5 seconds to achieve less than 0.1% total CANbus utilization
        m_settings.setColorFramePeriod(0.5);
        m_settings.setProximityFramePeriod(0.5);
        m_settings.setDigoutFramePeriod(0.5);
        m_settings.setStatusFramePeriod(0.5);

        //As an example, we are going to update the proximity and color data very quickly
        //In your application, you should set these appropriately depending on environmental conditions
        //This is to show how we can detect at the full bandwidth of the sensor without high latency or can utilization
        m_settings.setProximityIntegrationPeriod(ProximityPeriod.k5ms);
        m_settings.setColorIntegrationPeriod(ColorPeriod.k25ms);

        //Ensure that proximity and color frames are not sent out automatically when the sensor data updates
        m_settings.setAlignProximityFramesToIntegrationPeriod(false);
        m_settings.setAlignColorFramesToIntegrationPeriod(false);

        //Sets digital output port 1 to use the digital logic system, setting it to normally closed
       // m_settings.setDigoutPinConfig(m_sensor.digout1().channelIndex(), DigoutPinConfig.kDigoutLogicNormallyClosed);
       m_settings.setDigoutPinConfig(m_sensor.digout1().channelIndex(), DigoutPinConfig.kDigoutLogicActiveHigh);
       
        //Sets the digout frame trigger to send when the digout goes high or low
        m_settings.setDigoutFrameTrigger(m_sensor.digout1().channelIndex(), DigoutFrameTrigger.kRisingAndFalling);

        //Save settings to device
        m_sensor.setSettings(m_settings);

        //Configure the digout slot to trigger when proximity is between 0 and 0.5
        m_sensor.digout1().configureSlots(new HSVDigoutConfig()
        .setMaxProximity(0.5)
        .setMinProximity(0)
        );

    //canandcolor.digout1().getValue() will be updated every time the sensor enters or exit a digout trigger state.
    }
    public String getColorHexString(){
       // Logger.recordOutput("CanandColor", null);
        return m_sensor.getColor().toWpilibColor().toHexString();
    }
    
}
 
