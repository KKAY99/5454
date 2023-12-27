package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import frc.robot.Constants;

public class SpindexSubsystem  extends SubsystemBase {
    private TalonSRX m_Spindex= new TalonSRX(Constants.spindex.motor);

  public SpindexSubsystem() {
  }

   public void run(double speed){
    m_Spindex.set (ControlMode.PercentOutput,speed);
   }

   public void stop(){
    m_Spindex.set(ControlMode.PercentOutput,0);
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
