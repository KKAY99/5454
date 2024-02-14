package frc.robot.utils;
import frc.robot.Constants;
import frc.robot.Constants.LEDConstants;
import frc.robot.subsystems.BlinkInSubsystem;

import java.util.GregorianCalendar;
import java.util.Set;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;

public class LED{
    private AddressableLED m_led;
    private AddressableLEDBuffer m_ledBuffer;

    private LEDConstants.LEDStates m_currentPrimaryLEDState;
    private LEDConstants.LEDStates m_currentSecondaryLEDState;
    private LEDConstants.LEDDisplayStates m_currentLEDDisplay;
  
    private BlinkInSubsystem m_blinkIn;
    private double m_pattern =-1;
    public LED(int blinkInPWMport, int port,int ledCount){
        m_blinkIn=new BlinkInSubsystem(blinkInPWMport);
        m_led=new AddressableLED(port);
        m_ledBuffer=new AddressableLEDBuffer(ledCount);

        m_led.setLength(m_ledBuffer.getLength());
        m_led.setData(m_ledBuffer);

        m_led.start();
    }

    private void UpdatePrimaryLEDS(LEDConstants.LEDStates ledState){
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
        if(ledState!=m_currentPrimaryLEDState){
            m_currentPrimaryLEDState=ledState;
            UpdatePrimaryLEDS(m_currentPrimaryLEDState);
        }
    }
    public void testBlinkIn(){
        m_pattern=-0.35;
        m_blinkIn.runled(m_pattern);
        System.out.println("testing BlinkIn" + m_pattern);
    }
    public void LEDPeriodic(){
    }
}
