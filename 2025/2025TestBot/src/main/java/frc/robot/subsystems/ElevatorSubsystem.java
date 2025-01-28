// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;





public class ElevatorSubsystem extends SubsystemBase {
  private SparkBase m_motor1;
  private SparkMax m_motor2;
  /** Creates a new SpinMotor. */
  public ElevatorSubsystem(int CanID_1, int CanID_2) {
    m_motor1 = new SparkMax(CanID_1, MotorType.kBrushless);
    m_motor2 = new SparkMax(CanID_2, MotorType.kBrushless);
    SparkMaxConfig config1=new SparkMaxConfig();
    config1.idleMode(IdleMode.kBrake);
    m_motor1.configure(config1, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    m_motor2.configure(config1, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
  }
  
  public void motor_run(double speed){
    m_motor1.set(speed);
   // m_motor2.set(-speed);
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
