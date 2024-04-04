package frc.robot.utilities;
import frc.robot.Constants;
import frc.robot.Constants.LEDConstants;
import frc.robot.Constants.LEDConstants.CANDLELEDStates;
import frc.robot.subsystems.BlinkInSubsystem;
import java.util.ArrayList;
import java.util.GregorianCalendar;
import java.util.Set;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.LarsonAnimation;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.LarsonAnimation.BounceMode;

public class LED{
    private AddressableLED m_led;
    private AddressableLEDBuffer m_ledBuffer;

    private LEDConstants.PrimaryLEDStates m_currentPrimaryLEDState;
    private LEDConstants.SecondaryLEDStates m_currentSecondaryLEDState;
    private LEDConstants.LEDDisplayStates m_currentLEDDisplay;
    private LEDConstants.CANDLELEDStates m_currentCANDLELEDState;

    private BlinkInSubsystem m_blinkIn;
    private CANdle m_candle;
    private int m_ledCount=0;
    
    public LED(int canID,String CanBUS,int ledCount){
           //CanBus means we are using the CANdle
            m_candle=new CANdle(canID,CanBUS);
            m_candle.configLEDType(LEDStripType.GRB);
            m_candle.configBrightnessScalar(0.25);
            m_currentCANDLELEDState=CANDLELEDStates.NOSTATE;
            m_ledCount=ledCount;
    }

    public LED(int blinkInPWMport, int port,int ledCount){
        m_blinkIn=new BlinkInSubsystem(blinkInPWMport);
        m_led=new AddressableLED(port);
        m_ledBuffer=new AddressableLEDBuffer(ledCount);

        m_led.setLength(m_ledBuffer.getLength());
        m_led.setData(m_ledBuffer);

        m_led.start();
    }
    
    public LED(int port,int ledCount){
        m_led=new AddressableLED(port);
        m_ledBuffer=new AddressableLEDBuffer(ledCount);

        m_led.setLength(m_ledBuffer.getLength());
        for(int i=0;i<100;i++){
            m_ledBuffer.setRGB(1,160,32,240);
        }
        m_led.setData(m_ledBuffer);
        //SetLEDColor(LEDConstants.LEDColors.PURPLE);
      
        m_led.start();
               
    }


    private void UpdatePrimaryLEDS(LEDConstants.PrimaryLEDStates ledState){
        if(ledState!=LEDConstants.PrimaryLEDStates.OFF){
            m_led.start();
        }
        
        switch(ledState){
            case TELEOP:
            SetLEDColor(LEDConstants.LEDColors.GREEN);
            m_currentLEDDisplay=LEDConstants.LEDDisplayStates.SOLID;
            break;
            case AUTO:
            SetLEDColor(LEDConstants.LEDColors.RED);
            m_currentLEDDisplay=LEDConstants.LEDDisplayStates.SOLID;
            break;
            case TARGETLOCK:
            SetLEDColor(LEDConstants.LEDColors.PURPLE);
            m_currentLEDDisplay=LEDConstants.LEDDisplayStates.FLASHING;
            break;
            case NOTARGET:
            SetLEDColor(LEDConstants.LEDColors.RED);
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

    private void UpdateCANdleLEDS(LEDConstants.CANDLELEDStates ledState){
        try{
            ArrayList<Integer> rgbValues=new ArrayList<>();
            int startIndex=0;
            int secondStartIndex=0;
            int ledCount=0;
            int secondledCount=0;
            boolean noanimation=true;
            switch(ledState){
                case TARGETLOCK:
                rgbValues=getCANdleLEDColors(LEDConstants.LEDColors.PURPLE);
                startIndex=LEDConstants.candleTargetLockIndex1;
                ledCount=LEDConstants.candleLEDTargetLockLength1;
                secondStartIndex=LEDConstants.candleTargetLockIndex2;
                secondledCount=LEDConstants.candleLEDTargetLockLength2;
                break;
                case ROBOTFRONTSIDE:
                rgbValues=getCANdleLEDColors(LEDConstants.LEDColors.GREEN);
                startIndex=LEDConstants.candleRobotFrontStartIndex;
                secondStartIndex=LEDConstants.candleRobotFrontSecondStartIndex;
                ledCount=LEDConstants.candleRobotFrontLEDCount;
                secondledCount=LEDConstants.candleRobotFrontLEDCount2;
                break;
                case NOTARGET:
                rgbValues=getCANdleLEDColors(LEDConstants.LEDColors.RED);
                    startIndex=LEDConstants.candleTargetLockIndex1;
                    ledCount=LEDConstants.candleLEDTargetLockLength1;
                    secondStartIndex=LEDConstants.candleTargetLockIndex2;
                    secondledCount=LEDConstants.candleLEDTargetLockLength2;
                break;
                case AMPSCORESIDE:
                rgbValues=getCANdleLEDColors(LEDConstants.LEDColors.ORANGE);
                startIndex=LEDConstants.candleAmpScoreIndex;
                ledCount=LEDConstants.candleLEDAmpScoreLength;
                break;
                case DISABLED:
                   LarsonAnimation animation=new LarsonAnimation(160,32,240,0,0.8,
                        LEDConstants.candleLEDCount,BounceMode.Center,7);
                    //LarsonAnimation animation=new LarsonAnimation(160,32,240,0,0.8,
                    //    150,BounceMode.Center,7);
                //RainbowAnimation animation = new RainbowAnimation(1,1,50);  
                System.out.println("Starting Animation -" + ledState);
                    m_candle.animate(animation,0);
                    

                    
                noanimation=false;
                    break;
                case NOSTATE:
                    break;
            }
            if(noanimation && ledState!=CANDLELEDStates.NOSTATE){
                    m_candle.clearAnimation(0);
                    m_candle.setLEDs(rgbValues.get(0),rgbValues.get(1),
                                    rgbValues.get(2),rgbValues.get(3),startIndex,ledCount);
                    if(secondStartIndex!=0){
                        m_candle.setLEDs(rgbValues.get(0),rgbValues.get(1),
                                    rgbValues.get(2),rgbValues.get(3),secondStartIndex,
                                    secondledCount);
                    }
            }
        }
     catch (Exception e){
        //DO NOTHING
    
     }
    }
    private ArrayList<Integer> getCANdleLEDColors(LEDConstants.LEDColors ledColor){
        ArrayList<Integer> rgbValues=new ArrayList<>();
        switch(ledColor){
            case RED:
            //Red Value
            rgbValues.add(255);
            //Green Value
            rgbValues.add(0);
            //Blue Value
            rgbValues.add(0);
            //White Value
            rgbValues.add(0);
            break;
            case ORANGE:
            rgbValues.add(255);
            rgbValues.add(165);
            rgbValues.add(0);
            rgbValues.add(0);
            break;
            case BLUE:
            rgbValues.add(0);
            rgbValues.add(0);
            rgbValues.add(255);
            rgbValues.add(0);
            break;
            case PURPLE:
            rgbValues.add(160);
            rgbValues.add(32);
            rgbValues.add(240);
            rgbValues.add(0);
            break;
            case GREEN:
            rgbValues.add(0);
            rgbValues.add(255);
            rgbValues.add(0);
            rgbValues.add(0);
            break;
        }

        return rgbValues;
    }

    public void SetLEDPrimaryState(LEDConstants.PrimaryLEDStates ledState){
        if(ledState!=m_currentPrimaryLEDState){
            m_currentPrimaryLEDState=ledState;
                UpdatePrimaryLEDS(m_currentPrimaryLEDState); 
        }
    }

    public void SetCANdleLEDS(LEDConstants.CANDLELEDStates ledState){
     
        if(ledState!=m_currentCANDLELEDState){
                UpdateCANdleLEDS(ledState); 
                m_currentCANDLELEDState=ledState;     
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

