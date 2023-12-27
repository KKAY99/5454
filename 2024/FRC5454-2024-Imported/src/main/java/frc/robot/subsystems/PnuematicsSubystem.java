package frc.robot.subsystems;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class PnuematicsSubystem extends SubsystemBase{
  /** Creates a new ExampleSubsystem. */
  private edu.wpi.first.wpilibj.Compressor m_Compressor;
  private PneumaticHub m_hub;

  public PnuematicsSubystem(int nodeID,PneumaticsModuleType pModule) {
  try{
    m_hub=new PneumaticHub(nodeID);
    m_Compressor = new Compressor(nodeID,pModule);

  } catch (Exception e){
    System.out.println("Pneumatics Failure");
    System.out.println("Exception Message: " + e.getMessage());
    System.out.println("StackTrace:" + e.getStackTrace().toString());
  }
}
   @Override
   public void periodic() {
  
  }
 
   @Override
   public void simulationPeriodic() {
     // This method will be called once per scheduler run during simulation
   }

   public void clearPnuematicStickyFaults(){
    m_hub.clearStickyFaults();
   }
 }    





   