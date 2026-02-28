// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utilities.ObsidianCANSparkMax;

import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class IntakeSubsystem extends SubsystemBase {
  private TalonFX m_intakeMotor;
  private ObsidianCANSparkMax m_fold;
  private DutyCycleEncoder m_encoder;
  //private SparkAbsoluteEncoder m_encoder;

  public IntakeSubsystem(int CanId1, int CanId2) {
    m_intakeMotor = new TalonFX(CanId1);
    m_fold = new ObsidianCANSparkMax(CanId2, MotorType.kBrushless, true,40);
  }

  public void outFold(double speed) {
    m_fold.set(speed);
  }

  public void inFold(double speed) {
    m_fold.set(-speed);
  }

  public void stopFold() {
    m_fold.stopMotor();
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

  public double getFoldState() {
    return m_fold.getOutputCurrent();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Kicker Sensor", m_fold.getOutputCurrent());
    // This method will be called once per scheduler run
  }
}