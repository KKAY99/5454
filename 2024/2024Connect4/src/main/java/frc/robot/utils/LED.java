package frc.robot.utils;
import frc.robot.Constants;
import frc.robot.Constants.LEDConstants;
import frc.robot.Constants.LEDConstants.LEDStates;
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
        m_currentPrimaryLEDState=LEDStates.OFF;
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
            SetLEDColor(LEDConstants.LEDColors.PURPLE);
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
        int r=0;
        int g=0;
        int b=0;

        switch(ledColor){
            case RED:
            r=255;
            break;
            case ORANGE:
            r=255;
            g=165;
            break;
            case BLUE:
            b=255;
            break;
            case PURPLE:
            r=160;
            g=32;
            b=240;
            break;
            case GREEN:
            g=255;
            
            break;
        }

        for(int i=0;i<m_ledBuffer.getLength();i++){
            m_ledBuffer.setRGB(i,r,g,b);
        }
    }

    public void SetLEDState(LEDConstants.LEDStates ledState){
        System.out.println("Setting Led Color To: "+ledState);
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
