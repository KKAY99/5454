package frc.robot.utilities;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants.LaserCanConstants;

public class ADABreakBeam{
    public DigitalInput m_lowBreakBeam;
    public DigitalInput m_highBreakBeam;

    public ADABreakBeam(int lowDioPort,int highDioPort){
        m_lowBreakBeam=new DigitalInput(lowDioPort);
        //m_highBreakBeam=new DigitalInput(highDioPort);
    }

    public boolean getLowBreakBeam(){
        return m_lowBreakBeam.get();
    }

    public boolean getHighBreakBeam(){
        return m_highBreakBeam.get();
    }
}