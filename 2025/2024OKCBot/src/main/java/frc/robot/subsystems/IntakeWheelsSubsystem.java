// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;





public class IntakeWheelsSubsystem extends SubsystemBase {
  private WPI_TalonSRX m_motor;
  /** Creates a new SpinMotor. */
  public IntakeWheelsSubsystem(int CanID) {
    m_motor = new WPI_TalonSRX(CanID);
 
  }
  
  public void motor_run(double speed){
    m_motor.set(speed);

  }

  public void motor_stop(){
    m_motor.stopMotor();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
