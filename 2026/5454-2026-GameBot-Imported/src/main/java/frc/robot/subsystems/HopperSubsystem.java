package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utilities.ObsidianCANSparkMax;

public class HopperSubsystem extends SubsystemBase {
  private ObsidianCANSparkMax m_hopperMotor;
 
  //private SparkAbsoluteEncoder m_encoder;

  public HopperSubsystem(int CanId1) {
    m_hopperMotor = new ObsidianCANSparkMax(CanId1, MotorType.kBrushless, true);
   }

  public void agitate(double speed) {
    System.out.println("Running Agitate at " + speed);
    m_hopperMotor.set(speed);
  }

  public void stopAgitate(){
    m_hopperMotor.stopMotor();
  }
}
