package frc.robot.utilities;
import frc.robot.Constants;
import frc.robot.Constants.LEDConstants;
import frc.robot.subsystems.BlinkInSubsystem;

import java.util.GregorianCalendar;
import java.util.Set;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;

public class LED{
    private AddressableLED m_led;
    private AddressableLEDBuffer m_ledBuffer;

    private LEDConstants.PrimaryLEDStates m_currentPrimaryLEDState;
    private LEDConstants.SecondaryLEDStates m_currentSecondaryLEDState;
    private LEDConstants.LEDDisplayStates m_currentLEDDisplay;
  
    private BlinkInSubsystem m_blinkIn;

    public LED(int blinkInPWMport, int port,int ledCount){
        m_blinkIn=new BlinkInSubsystem(blinkInPWMport);
        m_led=new AddressableLED(port);
        m_ledBuffer=new AddressableLEDBuffer(ledCount);

        m_led.setLength(m_ledBuffer.getLength());
        m_led.setData(m_ledBuffer);

        m_led.start();
    }
    
    public LED( int port,int ledCount){
        m_led=new AddressableLED(port);
        m_ledBuffer=new AddressableLEDBuffer(ledCount);

        m_led.setLength(m_ledBuffer.getLength());
        for(int i=0;i<100;i++){
            m_ledBuffer.setRGB(1,160,32,240);
        }
        m_led.setData(m_ledBuffer);
        // SetLEDColor(LEDConstants.LEDColors.PURPLE);
      
        m_led.start();
               
    }


    private void UpdatePrimaryLEDS(LEDConstants.PrimaryLEDStates ledState){
        if(ledState!=LEDConstants.PrimaryLEDStates.OFF){
            m_led.start();
        }
        
        switch(ledState){
            case INTAKELOW:
            SetLEDColor(LEDConstants.LEDColors.RED);
            m_currentLEDDisplay=LEDConstants.LEDDisplayStates.FLASHING;
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
            case TARGETLOCK:
                SetLEDColor(LEDConstants.LEDColors.PURPLE);
                m_currentLEDDisplay=LEDConstants.LEDDisplayStates.FLASHING;
            
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

    private void UpdateSecondaryLEDS(LEDConstants.SecondaryLEDStates ledState){
        if(ledState!=LEDConstants.SecondaryLEDStates.OFF){
            m_led.start();
        }
        
        switch(ledState){
            case TARGETVISIBLE:
            m_blinkIn.runled(LEDConstants.blinkinYellow);
            break;
            case TARGETLOCK:
            m_blinkIn.runled(LEDConstants.blinkinGreen);
            break;
            case NOTARGET:
            m_blinkIn.runled(LEDConstants.blinkinRed);
            break;
            case OFF:
            m_blinkIn.stopled();
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

    public void SetLEDPrimaryState(LEDConstants.PrimaryLEDStates ledState){
        if(ledState!=m_currentPrimaryLEDState){
            m_currentPrimaryLEDState=ledState;
            UpdatePrimaryLEDS(m_currentPrimaryLEDState);
        }
    }

    public void SetLEDSecondaryState(LEDConstants.SecondaryLEDStates ledState){
        if(ledState!=m_currentSecondaryLEDState){
            m_currentSecondaryLEDState=ledState;
            UpdateSecondaryLEDS(m_currentSecondaryLEDState);
        }
    }

    public void LEDPeriodic(){
        Logger.recordOutput("LED/CurrentPrimaryLEDState",m_currentPrimaryLEDState);
        Logger.recordOutput("LED/CurrentSecondaryLEDState",m_currentPrimaryLEDState);
        Logger.recordOutput("LED/CurrentLEDDisplayState",m_currentLEDDisplay);
    }
}
