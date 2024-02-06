package frc.robot.utilities;
import frc.robot.Constants.LEDConstants;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;

public class LED{
    private AddressableLED m_led;
    private AddressableLEDBuffer m_ledBuffer;

    private LEDConstants.LEDCOLORS m_currentLEDColor;

    public LED(int port,int ledCount){
        m_led=new AddressableLED(port);
        m_ledBuffer=new AddressableLEDBuffer(ledCount);

        m_led.setLength(m_ledBuffer.getLength());
        m_led.setData(m_ledBuffer);

        m_led.start();
    }

    public void SetLEDColor(LEDConstants.LEDCOLORS ledColor){
        switch(ledColor){
            case RED:
            m_ledBuffer.setRGB(0,255,0,0);
            break;
            case GREEN:
            m_ledBuffer.setRGB(0,0,255,0);
            break;
            case PURPLE:
            m_ledBuffer.setRGB(0,160,32,240);
            break;
            case ORANGE:
            m_ledBuffer.setRGB(0,255,165,0);
            break;
        }
    }

    public void SetLEDColorIntake(){
        m_currentLEDColor=LEDConstants.LEDCOLORS.RED;
    }

    public void SetLEDColorNoteReady(){
        m_currentLEDColor=LEDConstants.LEDCOLORS.ORANGE;
    }

    public void updateLEDs(){
        SetLEDColor(m_currentLEDColor);
    }
}
