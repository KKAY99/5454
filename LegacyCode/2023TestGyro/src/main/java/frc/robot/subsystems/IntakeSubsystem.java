package frc.robot.subsystems;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem  extends SubsystemBase {
  private Spark m_Intake= new Spark(Constants.intake.motorPWM);

  public IntakeSubsystem() {

  }

   public void run(double speed){
    m_Intake.set(speed);
   }

   public void stop(){
    m_Intake.set(0);
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
