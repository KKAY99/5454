package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import frc.robot.Constants;

public class SpindexSubsystem  extends SubsystemBase {
    private Spark m_Spindex= new Spark(Constants.spindex.motorPWM);

  public SpindexSubsystem() {
  }

   public void run(double speed){
    m_Spindex.set (speed);
   }

   public void stop(){
    m_Spindex.set(0);
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
