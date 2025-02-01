package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import frc.robot.utilities.ObsidianCANSparkMax;
import com.revrobotics.spark.*;

public class DunkinDonutSubsystem extends SubsystemBase {
  private ObsidianCANSparkMax m_motor1;
  private ObsidianCANSparkMax m_motor2;
  private ObsidianCANSparkMax m_rotateMotor;

  private SparkClosedLoopController m_loopController;
  
  public DunkinDonutSubsystem(int CanID_1, int CanID_2, int rotateCanID) {
    m_motor1 = new ObsidianCANSparkMax(CanID_1, MotorType.kBrushless, true);
    m_motor2 = new ObsidianCANSparkMax(CanID_2, MotorType.kBrushless, true);
    m_rotateMotor = new ObsidianCANSparkMax(rotateCanID, MotorType.kBrushless, true);

    m_loopController=m_rotateMotor.getClosedLoopController();
  }

  public double get_rotatemotorpos(){
    return m_rotateMotor.getEncoder().getPosition();
     
  }

  public void set_referance(double pos){
    m_loopController.setReference(pos, ControlType.kPosition);
  }

  public void reset_referamce(){
    m_loopController.setReference(0, ControlType.kVelocity);
  }

  public void run_rotatemotor(double speed){
    m_rotateMotor.set(speed);
  }

  public void stop_rotatemotor(){
    m_rotateMotor.stopMotor();
  }
  
  public void motor_runmotor(int motorNumber,double speed){
    if(motorNumber==1){
      m_motor1.set(speed);
   } else {
     m_motor2.set(speed);
   }
  }

  public void motor_stop(int motorNumber){
    if(motorNumber==1){
      m_motor1.stopMotor();
   } else {
     m_motor2.stopMotor();
   }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
