// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;





public class SpinMotor extends SubsystemBase {
  private SparkMax m_motor1;
  //private SparkMax m_motor2;
  /** Creates a new SpinMotor. */
  public SpinMotor(int CanID_1/*, int CanID_2*/) {
    m_motor1 = new SparkMax(CanID_1, MotorType.kBrushless);
    //m_motor2 = new SparkMax(CanID_2, MotorType.kBrushless);

  }
  
  public void motor_run(double speed){
    m_motor1.set(speed);
    //m_motor2.set(speed);
  }

  public void motor_stop(){
    m_motor1.stopMotor();
    //m_motor2.stopMotor();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
