package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.Constants.ClimbConstants;
import frc.robot.Constants.TurretConstants;
import frc.robot.utilities.ObsidianCANSparkMax;

public class ClimbSubsystem extends SubsystemBase {
  private ObsidianCANSparkMax m_climbMotor;
  private DigitalInput m_climbUpSwitch;
  private DigitalInput m_climbDownSwitch;
 
  //private SparkAbsoluteEncoder m_encoder;

  public ClimbSubsystem(int CanId1) {
    m_climbMotor = new ObsidianCANSparkMax(CanId1, MotorType.kBrushless, true);
    m_climbUpSwitch = new DigitalInput(ClimbConstants.climbUpSwitchDIO);
    m_climbDownSwitch = new DigitalInput(ClimbConstants.climbDownSwitchDIO);
  }

  public boolean isClimbUpLimit() {
    return m_climbUpSwitch.get();
  }

  public boolean isClimbDownLimit() {
    return m_climbDownSwitch.get();
  }

  public void climbGo(double speed) {
    m_climbMotor.set(speed);
  }

  public void stopClimb(){
    m_climbMotor.stopMotor();
  }

  public void extendClimb() {
    m_climbMotor.set(ClimbConstants.climbForwardSpeed);
    if (isClimbUpLimit()) {
      m_climbMotor.stopMotor();
    }
  }
  public void retractClimb() {
    m_climbMotor.set(ClimbConstants.climbBackSpeed);
    if (isClimbDownLimit()) {
      m_climbMotor.stopMotor();
    }
  }

  public Command climbUpCommand() {
    return Commands.runOnce(    ()->extendClimb(),this);
  }

  public Command climbDownCommand() {
    return Commands.runOnce(    ()->retractClimb(),this);
  }

}