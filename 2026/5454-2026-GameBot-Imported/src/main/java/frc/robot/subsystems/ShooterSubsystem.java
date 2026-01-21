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

public class ShooterSubsystem extends SubsystemBase {
  private ObsidianCANSparkMax m_shooterMotor;

  public ShooterSubsystem(int CanId1) {
    m_shooterMotor = new ObsidianCANSparkMax(CanId1, MotorType.kBrushless, true);
  }

  public void runIntake(double speed,double lowspeed) {
    m_shooterMotor.set(speed);
  }

  public void stopIntake(){
    m_shooterMotor.stopMotor();
  }

}