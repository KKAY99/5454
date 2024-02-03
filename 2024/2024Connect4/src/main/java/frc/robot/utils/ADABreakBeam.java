package frc.robot.utils;
import au.grapplerobotics.LaserCan;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants.LaserCanConstants;

public class ADABreakBeam{
    public DigitalInput m_breakBeam;

    public ADABreakBeam(int dioPort){
        m_breakBeam=new DigitalInput(dioPort);
    }

    public boolean BreakBeam(){
        return m_breakBeam.get();
    }
}