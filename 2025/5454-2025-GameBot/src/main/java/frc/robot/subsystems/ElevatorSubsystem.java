package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import frc.robot.utilities.ObsidianCANSparkMax;

public class ElevatorSubsystem extends SubsystemBase {
  private ObsidianCANSparkMax m_motor1;
  private SparkClosedLoopController m_loopController;

  public ElevatorSubsystem(int CanID_1) {
    m_motor1 = new ObsidianCANSparkMax(CanID_1, MotorType.kBrushless, true);
    m_loopController=m_motor1.getClosedLoopController();

  }

  public double get_motor1pos(){
    return m_motor1.getEncoder().getPosition();
  }

  public void set_referance(double pos){
    m_loopController.setReference(pos, ControlType.kPosition);
  }

  public void reset_referamce(){
    m_loopController.setReference(0, ControlType.kVelocity);
  }
  
  public void motor_run(double speed){
    m_motor1.set(speed);
  }

  public void motor_stop(){
    m_motor1.stopMotor();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
