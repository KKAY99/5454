package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utilities.ObsidianCANSparkMax;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;

public class ElevatorSubsystem extends SubsystemBase {
  private TalonFX m_motor1;
  private TalonFX m_motor2;

  private PositionDutyCycle m_posDutyCycle;
  private VelocityDutyCycle m_VelocityDutyCycle;

  private double MaxVeloc = 7530;

  public ElevatorSubsystem(int Motor1ID, int Motor2ID){
    m_motor1 = new TalonFX(Motor1ID);
    m_motor2 = new TalonFX(Motor2ID);
  }

  public void run(double speed){
    m_VelocityDutyCycle = new VelocityDutyCycle(speed*MaxVeloc);
    m_motor1.setControl(m_VelocityDutyCycle);
    m_motor2.setControl(m_VelocityDutyCycle);
  }

  public void stop(){
    m_motor1.stopMotor();
    m_motor2.stopMotor();
  }

  public void setPos(double pos){
    m_posDutyCycle = new PositionDutyCycle(pos);
    m_motor1.setControl(m_posDutyCycle);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
