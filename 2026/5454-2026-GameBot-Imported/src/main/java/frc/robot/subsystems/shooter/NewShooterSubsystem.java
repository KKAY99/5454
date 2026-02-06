// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

public class NewShooterSubsystem extends SubsystemBase {
    private TalonFX m_1shooterMotor;
    private TalonFX m_2shooterMotor;
    private TalonFX m_kickerMotor;

  public NewShooterSubsystem(int shooter1CANID, int shooter2CANID, int kickerCANID) {
    m_1shooterMotor = new TalonFX(shooter1CANID);
    m_2shooterMotor = new TalonFX(shooter2CANID);
    m_kickerMotor = new TalonFX(kickerCANID);
  }

  public void runNewShooter(double speed) {
    m_1shooterMotor.set(speed);
    m_2shooterMotor.set(-speed);
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
    m_1shooterMotor.stopMotor();
    m_2shooterMotor.stopMotor();
  }

  public void moveHood(double speed){
    m_kickerMotor.set(speed);
  }

  public void stopHood(){
    m_kickerMotor.stopMotor();
  }

}