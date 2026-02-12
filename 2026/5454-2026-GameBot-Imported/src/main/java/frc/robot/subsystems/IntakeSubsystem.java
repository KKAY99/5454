// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utilities.ObsidianCANSparkMax;

import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

public class IntakeSubsystem extends SubsystemBase {
  private ObsidianCANSparkMax m_intakeMotor;
  private ObsidianCANSparkMax m_lowMotor;
  private DutyCycleEncoder m_encoder;
  //private SparkAbsoluteEncoder m_encoder;

  public IntakeSubsystem(int CanId1) {
    m_intakeMotor = new ObsidianCANSparkMax(CanId1, MotorType.kBrushless, true,80);
  }

  public void runIntake(double speed) {
    m_intakeMotor.set(speed);
  }

  public void stopIntake(){
    m_intakeMotor.stopMotor();
  }
  public Command outtakeCommand(){
      return Commands.startEnd(    ()->runIntake(IntakeConstants.outtakeSpeed),
                                          ()->stopIntake(),
                                          this);
  }
  
  public Command intakeCommand(){
      return Commands.startEnd(    ()->runIntake(IntakeConstants.highSpeed),
                                          ()->stopIntake(),
                                          this);
  }
  public Command intakeonCommand(){
    return Commands.runOnce(    ()->runIntake(IntakeConstants.highSpeed),this);
  }
  public Command intakeoffCommand(){
    return Commands.runOnce(    ()->stopIntake(),this);
  }
}