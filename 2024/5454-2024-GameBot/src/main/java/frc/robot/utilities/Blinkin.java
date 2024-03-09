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
import frc.robot.Constants.BlinkinConstants;

public class Blinkin{
    private BlinkinConstants.LEDStates m_LEDState;
    private BlinkInSubsystem m_blinkIn;

    public Blinkin(int blinkInPWMport){
        m_blinkIn=new BlinkInSubsystem(blinkInPWMport);
    }

    private void UpdateBlinkin(BlinkinConstants.LEDStates ledState){
        switch(ledState){
            case ISATLIMIT:
            SetLEDColor(BlinkinConstants.LEDColors.ORANGE);
            break;
            case NOTARGET:
            SetLEDColor(BlinkinConstants.LEDColors.RED);
            break;
            case AUTO:
            SetLEDColor(BlinkinConstants.LEDColors.RED);
            break;
            case INTAKEHASNOTE:
            SetLEDColor(BlinkinConstants.LEDColors.ORANGE);
            break;
            case TARGETLOCK:
            SetLEDColor(BlinkinConstants.LEDColors.GREEN);
            break;
            case OFF:
            m_blinkIn.runled(0);
            break;
        }
    }

    private void SetLEDColor(BlinkinConstants.LEDColors ledColor){
        switch(ledColor){
            case RED:
            m_blinkIn.runled(BlinkinConstants.red);
            break;
            case ORANGE:
            m_blinkIn.runled(BlinkinConstants.orange);
            break;
            case BLUE:
            m_blinkIn.runled(BlinkinConstants.blue);
            break;
            case PURPLE:
            m_blinkIn.runled(BlinkinConstants.purple);
            break;
            case GREEN:
            m_blinkIn.runled(BlinkinConstants.green);
            break;
        }
    }

    public void SetLEDPrimaryState(BlinkinConstants.LEDStates ledState){
        if(ledState!=m_LEDState){
            m_LEDState=ledState;
            UpdateBlinkin(m_LEDState);
        }
    }

    public void LEDPeriodic(){
    }
}

