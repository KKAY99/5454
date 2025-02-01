// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import frc.robot.utilities.ObsidianCANSparkMax;

public class IntakeSubsystem extends SubsystemBase {
  private ObsidianCANSparkMax m_motor1;
  private ObsidianCANSparkMax m_motor2;
  /** Creates a new SpinMotor. */
  public IntakeSubsystem(int CanID_1, int CanID_2) {
    m_motor1 = new ObsidianCANSparkMax(CanID_1, MotorType.kBrushless, true);
    m_motor2 = new ObsidianCANSparkMax(CanID_2, MotorType.kBrushless, true);

  }
  
  public void motor_run(double speed){
    m_motor1.set(-speed);
    m_motor2.set(speed);
  }

  public void motor_stop(){
    m_motor1.stopMotor();
    m_motor2.stopMotor();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
