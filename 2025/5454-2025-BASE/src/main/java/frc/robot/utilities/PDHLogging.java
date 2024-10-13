package frc.robot.utilities;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
public class PDHLogging {
 
    private PowerDistribution m_PDH;
    public PDHLogging(int PDHID){
        m_PDH = new PowerDistribution(PDHID, ModuleType.kRev);
    }
public void updatePDHLog(){
    for (int breakerloop=0;breakerloop<=23;breakerloop++){
    Logger.recordOutput("PDH/CurrentPort"+ breakerloop,m_PDH.getCurrent(breakerloop));
    }
    Logger.recordOutput("PDH/TotalOutput",m_PDH.getTotalCurrent());

}
}
