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
import com.ctre.phoenix6.hardware.CANcoder;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import yams.units.EasyCRT;

public class TurretSubsystem extends SubsystemBase {
  private ObsidianCANSparkMax m_turretMotor;
  private CANcoder m_encoder1;
  private CANcoder m_encoder2;
  //private SparkAbsoluteEncoder m_encoder;

  public TurretSubsystem(int CanId1, int encoder1, int encoder2) {
    m_turretMotor = new ObsidianCANSparkMax(CanId1, MotorType.kBrushless, true);
   }

  public void moveTurret(double speed) {
    m_turretMotor.set(speed);
  }

  public void stopTurret(){
    m_turretMotor.stopMotor();
  }

}