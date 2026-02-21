// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utilities.ObsidianCANSparkMax;

import frc.robot.Constants;

import edu.wpi.first.wpilibj.DutyCycleEncoder;

public class ShooterSubsystem extends SubsystemBase {
  private ObsidianCANSparkMax m_shooterMotor;
  private ObsidianCANSparkMax m_kickerMotor;
  private double m_idleSpeed;

  public ShooterSubsystem(int shooterCANID,int kickerCANID, double idleSpeed) {
    m_shooterMotor = new ObsidianCANSparkMax(shooterCANID, MotorType.kBrushless, true);
    m_kickerMotor = new ObsidianCANSparkMax(kickerCANID, MotorType.kBrushless, true);
    m_idleSpeed=idleSpeed;
  }

  public void setShooterIdleSpin(){
    //use power setting to avoid PID loops for idle
    m_shooterMotor.set(m_idleSpeed);

  }
  
  public void runShooter(double speed,double kickerSpeed) {
    System.out.println("Shooter Speed Set " + speed);
    m_shooterMotor.set(speed);
    m_kickerMotor.set(kickerSpeed);
  }

  public void stopShooter(){
    System.out.println("Shooter Stopped");
    m_shooterMotor.stopMotor();
    m_kickerMotor.stopMotor();
  }

  public Command OldShootCommand(){
    return Commands.startEnd(     ()->runShooter(Constants.ShooterConstants.shootSpeed, Constants.ShooterConstants.KickerSpeed),
                                  ()->stopShooter(),
                                  this);
  }
  public Command OldShootonCommand(){
    return Commands.runOnce(    ()->runShooter(Constants.ShooterConstants.shootSpeed, Constants.ShooterConstants.KickerSpeed),this);
  }
  public Command OldShootoffCommand(){
    return Commands.runOnce(    ()->stopShooter(),this);
  }



}