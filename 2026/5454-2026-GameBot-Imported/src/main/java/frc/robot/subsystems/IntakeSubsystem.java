// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utilities.ObsidianCANSparkMax;

import frc.robot.Constants;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

public class IntakeSubsystem extends SubsystemBase {
  private ObsidianCANSparkMax m_intakeMotor;
  private ObsidianCANSparkMax m_lowMotor;
  private DutyCycleEncoder m_encoder;
  //private SparkAbsoluteEncoder m_encoder;

  public IntakeSubsystem(int CanId1, int CanId2) {
    m_intakeMotor = new ObsidianCANSparkMax(CanId1, MotorType.kBrushless, true,80);
   
    m_lowMotor = new ObsidianCANSparkMax(CanId2, MotorType.kBrushless, true);
  }

  public void runIntake(double speed,double lowspeed) {
    m_intakeMotor.set(speed);
    m_lowMotor.set(lowspeed);
  }

  public void stopIntake(){
    m_intakeMotor.stopMotor();
    m_lowMotor.stopMotor();
  }

}