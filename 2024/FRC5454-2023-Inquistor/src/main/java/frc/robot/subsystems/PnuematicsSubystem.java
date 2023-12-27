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
  private double m_pressure; 
  private Solenoid m_solenoidClaw;
  private Solenoid m_solenoidPunch;
  private PneumaticHub m_hub;
  public PnuematicsSubystem(int nodeID,PneumaticsModuleType pModule, int clawSolenoid,int conveyorPunch) {
  try{
    m_hub=new PneumaticHub(nodeID);
    m_Compressor = new Compressor(nodeID,pModule);
    m_solenoidClaw =new Solenoid(nodeID,pModule,clawSolenoid);
   // m_solenoidClaw.setPulseDuration(1 ); // KK was 2
    m_solenoidPunch =new Solenoid(nodeID,pModule,conveyorPunch);
    m_Compressor.enableDigital();
    m_pressure=m_Compressor.getPressure();

  } catch (Exception e){
    System.out.println("Pneumatics Failure");
    System.out.println("Exception Message: " + e.getMessage());
    System.out.println("StackTrace:" + e.getStackTrace().toString());
  }
}
  public void setConveyorPunch(boolean value){
    m_solenoidPunch.set(value);
  }
  public boolean getConveyorPunch(){
    return m_solenoidPunch.get();
  }

  public void setClaw(boolean value){
 System.out.println("CLAW SET " + value  );
    m_solenoidClaw.set(value);
  
  }
  public void openClaw(){
    setClaw(true);   
     }
  public boolean getClaw(){
  //return true;
     return m_solenoidClaw.get();
  }
  public double getPressure(){
  
      return m_Compressor.getPressure();
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





   