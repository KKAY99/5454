// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import frc.robot.Constants.HoodConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.utilities.ObsidianCANSparkMax;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class NewShooterSubsystem extends SubsystemBase {
    private TalonFX m_1shooterMotor;
    private TalonFX m_2shooterMotor;
    private TalonFX m_hoodMotor;
    //private TalonFX m_kickerMotor;
    private ObsidianCANSparkMax m_kickerMotor;
  public NewShooterSubsystem(int shooter1CANID, int shooter2CANID, int kickerCANID,int hoodCANID) {
    m_1shooterMotor = new TalonFX(shooter1CANID);
    m_2shooterMotor = new TalonFX(shooter2CANID);
    m_hoodMotor = new TalonFX(hoodCANID);
    m_kickerMotor = new ObsidianCANSparkMax(kickerCANID, MotorType.kBrushless,true);
  }

  public void runNewShooter(double speed,double kickerSpeed) {
    System.out.println("Shooter Spin:" + speed);
    m_1shooterMotor.set(speed);
    m_2shooterMotor.set(-speed);
    m_kickerMotor.set(kickerSpeed);
  }

public void runShooterVelocity(double targetSpeed){
// 
   m_1shooterMotor.setControl(new VelocityTorqueCurrentFOC(0)
                  .withVelocity(targetSpeed)
                  .withFeedForward(0.1));
    m_2shooterMotor.setControl(new VelocityTorqueCurrentFOC(0)
                  .withVelocity(-targetSpeed)
                  .withFeedForward(0.1));
   }
  

  public void stopNewShooter(){
    System.out.println("stopping shooter");
    m_1shooterMotor.stopMotor();
    m_2shooterMotor.stopMotor();
    m_kickerMotor.stopMotor();
  }

  public void moveHood(double speed){
    m_hoodMotor.set(speed);
  }

  public void stopHood(){
    m_hoodMotor.stopMotor();
  }

  public Command HoodUp(){
    return Commands.startEnd(    ()->moveHood(HoodConstants.hoodUpSpeed),
                                           ()->stopHood(),
                                           this);
  }
  public Command HoodDown(){
    return Commands.startEnd(    ()->moveHood(HoodConstants.hoodDownSpeed),
                                           ()->stopHood(),
                                           this);
  }
  public Command shootCommand(){
    return Commands.startEnd(    ()->runNewShooter(ShooterConstants.shootSpeed,
                                    ShooterConstants.KickerSpeed),
                                           ()->stopNewShooter(),
                                           this);
  }
  public Command shootonCommand(){
    return Commands.runOnce(    ()->runNewShooter(ShooterConstants.shootSpeed,
                                    ShooterConstants.KickerSpeed),this);
  }
  public Command shootoffCommand(){
    return Commands.runOnce(    ()->stopNewShooter(),this);
  }
}