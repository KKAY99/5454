// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

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
  private ObsidianCANSparkMax m_kickerMotor;

  public ShooterSubsystem(int shooterCANID,int kickerCANID) {
    m_shooterMotor = new ObsidianCANSparkMax(shooterCANID, MotorType.kBrushless, true);
    m_kickerMotor = new ObsidianCANSparkMax(kickerCANID, MotorType.kBrushless, true);
  }

  public void runShooter(double speed,double kickerSpeed) {
    m_shooterMotor.set(speed);
    m_kickerMotor.set(kickerSpeed);
  }

  public void stopShooter(){
    m_shooterMotor.stopMotor();
    m_kickerMotor.stopMotor();
  }

}