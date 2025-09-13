// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.classes.ObsidianCANSparkMax;


public class ClimbSubsystem extends SubsystemBase {

  private ObsidianCANSparkMax m_climbMotor;

  /** Creates a new Intake. */
  public ClimbSubsystem(int CanId) {
    m_climbMotor = new ObsidianCANSparkMax(CanId, MotorType.kBrushless, true);
  }


  public void run(double speed){
    m_climbMotor.set(speed); 
  }

  public void stop(){
    m_climbMotor.stopMotor();
  }
}
