package frc.robot.subsystems;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType; 
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClawSubsystem extends SubsystemBase {
 private static Solenoid m_solenoidClaw0;
 private static Solenoid m_solenoidClaw1;
 private static Solenoid m_solenoidClaw2;
 private static Solenoid m_solenoidClaw3;

 public static PneumaticsModuleType pModule = PneumaticsModuleType.CTREPCM;
  
  /** Creates a new ExampleSubsystem. */
  public ClawSubsystem() {
   Compressor compressor = new Compressor(11,pModule);
  m_solenoidClaw0 =new Solenoid(11,pModule, 0);
 
   //m_solenoidClaw1 =new Solenoid(pModule, 1);
   //m_solenoidClaw2 =new Solenoid(pModule, 2);
  // m_solenoidClaw3 =new Solenoid(pModule, 3);
   
   
  // setEnabled();
  }
  
  public void OpenClaw(){
    System.out.println("Open Claw");
    m_solenoidClaw0.set(false);
  //  m_solenoidClaw1.set(false);
  //  m_solenoidClaw2.set(false);
  //  m_solenoidClaw3.set(false);
 
  }
  public void CloseClaw(){
    System.out.println("Close Claw");
    m_solenoidClaw0.set(true);
   // m_solenoidClaw1.set(true);
   // m_solenoidClaw2.set(true);
   // m_solenoidClaw3.set(true);
 
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
