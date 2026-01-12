package frc.robot.utilities;

import java.util.function.BooleanSupplier;
import edu.wpi.first.wpilibj.DigitalOutput;

public class JacksonsCoolPanel {
    private DigitalOutput m_greenLED;
    private DigitalOutput m_redLED;
    
    public JacksonsCoolPanel(int greenPort,int redPort){
        m_greenLED=new DigitalOutput(greenPort);
        m_redLED=new DigitalOutput(redPort);
    }

    public void isAllCanAvailable(BooleanSupplier supplier){
        m_greenLED.set((supplier.getAsBoolean())?true:false);
        m_redLED.set((supplier.getAsBoolean())?false:true);
    }
}