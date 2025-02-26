package frc.robot.classes;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PWM;

public class BreakBeam {
    private DigitalInput m_breakBeam;

    public double m_breakDistance;

    public BreakBeam(int breakBeamPort, double breakDistance){
        m_breakBeam=new DigitalInput(breakBeamPort);
        m_breakDistance=breakDistance;
    }

    // public boolean GetBreakBeam(){
    //     return (m_breakBeam.isAnalogTrigger();<m_breakDistance);
    // }

    // public double GetDistance(){
    //     return m_breakBeam.getSpeed();
    // }
    
}
