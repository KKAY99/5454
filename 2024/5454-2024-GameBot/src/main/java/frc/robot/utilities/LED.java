package frc.robot.utilities;
import frc.robot.Constants;
import frc.robot.Constants.LEDConstants;

import java.util.GregorianCalendar;
import java.util.Set;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;

public class LED{
    private AddressableLED m_led;
    private AddressableLEDBuffer m_ledBuffer;

    private LEDConstants.LEDStates m_currentLEDState;
    private LEDConstants.LEDDisplayStates m_currentLEDDisplay;

    public LED(int port,int ledCount){
        m_led=new AddressableLED(port);
        m_ledBuffer=new AddressableLEDBuffer(ledCount);

        m_led.setLength(m_ledBuffer.getLength());
        m_led.setData(m_ledBuffer);

        m_led.start();
    }

    private void UpdateLEDS(LEDConstants.LEDStates ledState){
        if(ledState!=LEDConstants.LEDStates.OFF){
            m_led.start();
        }
        
        switch(ledState){
            case INTAKELOW:
            SetLEDColor(LEDConstants.LEDColors.RED);
            m_currentLEDDisplay=LEDConstants.LEDDisplayStates.FLASHING;
            break;
            case TARGETLOCK:
            SetLEDColor(LEDConstants.LEDColors.GREEN);
            m_currentLEDDisplay=LEDConstants.LEDDisplayStates.SOLID;
            break;
            case TELEOP:
            SetLEDColor(LEDConstants.LEDColors.GREEN);
            m_currentLEDDisplay=LEDConstants.LEDDisplayStates.SOLID;
            break;
            case AUTO:
            SetLEDColor(LEDConstants.LEDColors.RED);
            m_currentLEDDisplay=LEDConstants.LEDDisplayStates.SOLID;
            break;
            case INTAKEHASNOTE:
            SetLEDColor(LEDConstants.LEDColors.ORANGE);
            m_currentLEDDisplay=LEDConstants.LEDDisplayStates.FLASHING;
            break;
            case OFF:
            m_led.stop();
            break;
        }

        switch(m_currentLEDDisplay){
            case SOLID:
            m_led.setData(m_ledBuffer);
            break;
            case FLASHING:
            m_led.setData(m_ledBuffer);
            break;
            case CHASING:
            m_led.setData(m_ledBuffer);
            break;
        }
    }

    private void SetLEDColor(LEDConstants.LEDColors ledColor){
        switch(ledColor){
            case RED:
            m_ledBuffer.setRGB(m_ledBuffer.getLength(),255,0,0);
            break;
            case ORANGE:
            m_ledBuffer.setRGB(m_ledBuffer.getLength(),255,165,0);
            break;
            case BLUE:
            m_ledBuffer.setRGB(m_ledBuffer.getLength(),0,0,255);
            break;
            case PURPLE:
            m_ledBuffer.setRGB(m_ledBuffer.getLength(),160,32,240);
            break;
            case GREEN:
            m_ledBuffer.setRGB(m_ledBuffer.getLength(),0,255,0);
            break;
        }
    }

    public void SetLEDState(LEDConstants.LEDStates ledState){
        if(ledState!=m_currentLEDState){
            m_currentLEDState=ledState;
            UpdateLEDS(m_currentLEDState);
        }
    }

    public void LEDPeriodic(){
        Logger.recordOutput("LED/CurrentLEDState",m_currentLEDState);
        Logger.recordOutput("LED/CurrentLEDDisplayState",m_currentLEDDisplay);
    }
}
