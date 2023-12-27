package frc.robot.subsystems;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import frc.robot.Constants;

public class IntakeSubsystem  extends SubsystemBase {
  private TalonSRX m_Intake;
 public IntakeSubsystem(int canPort) {
 m_Intake=new TalonSRX(canPort);
 
  }

   public void run(double speed){
    m_Intake.set(ControlMode.PercentOutput,speed);
   }

   public void stop(){
    m_Intake.set(ControlMode.PercentOutput,0);
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
