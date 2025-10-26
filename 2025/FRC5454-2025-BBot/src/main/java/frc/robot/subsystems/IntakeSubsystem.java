// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.classes.ObsidianCANSparkMax;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

public class IntakeSubsystem extends SubsystemBase {
  private ObsidianCANSparkMax m_intakeMotor;
  private ObsidianCANSparkMax m_rotateMotor;
  private DutyCycleEncoder m_encoder;
  //private SparkAbsoluteEncoder m_encoder;

  public IntakeSubsystem(int CanId1, int CanId2, int encoder) {
    m_intakeMotor = new ObsidianCANSparkMax(CanId1, MotorType.kBrushless, true);
    m_rotateMotor = new ObsidianCANSparkMax(CanId2, MotorType.kBrushless, true, Constants.k20Amp);
    m_encoder = new DutyCycleEncoder(encoder);
    


  }

  public void runIntake(double speed) {
    m_intakeMotor.set(speed);
  }

  public Command score() {
    return new InstantCommand(() -> runIntake(1));
  }

  public void stopIntake(){
    m_intakeMotor.stopMotor();
  }

  public void Rotate(double speed){
    m_rotateMotor.set(speed);
  }

  public void stopRotate(){
    m_rotateMotor.stopMotor();
  }

  public double getRotatePosition(){
    //if returning negative short term hack is to make put a negative in fromt of this
    //return -m_rotateMotor.getEncoder().getPosition();
    return -
m_rotateMotor.getEncoder().getPosition();
  }
  public double getRotatePositionABS(){
    return (m_encoder.get());
  }

}