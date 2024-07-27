package frc.robot.classes;
import edu.wpi.first.wpilibj.DigitalInput;

public class BreakBeam {
    private DigitalInput m_breakBeam;

    public BreakBeam(int breakBeamPort){
        m_breakBeam=new DigitalInput(breakBeamPort);
    }

    public boolean GetBreakBeam(){
        return m_breakBeam.get();
    }
    
}
