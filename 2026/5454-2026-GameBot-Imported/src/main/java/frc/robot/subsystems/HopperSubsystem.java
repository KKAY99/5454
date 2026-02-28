package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utilities.ObsidianCANSparkMax;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;

public class HopperSubsystem extends SubsystemBase {
  private ObsidianCANSparkMax m_hopperMotor;
  private DigitalInput m_BREAK;
 
  //private SparkAbsoluteEncoder m_encoder;

  public HopperSubsystem(int CanId1, int fuelSensorDIO) {
    m_hopperMotor = new ObsidianCANSparkMax(CanId1, MotorType.kBrushless, true, Constants.k70Amp);
    m_BREAK = new DigitalInput(fuelSensorDIO);
   }

  public void agitate(double speed) {
    System.out.println("Running Agitate at " + speed);
    m_hopperMotor.set(speed);
  }

  public void stopAgitate(){
    m_hopperMotor.stopMotor();
  }
  public Command agitateCommand(){
    return Commands.startEnd(    ()->agitate(Constants.HopperConstants.agitateSpeed),
                                           ()->stopAgitate(),
                                           this);

  }
  public Command agitateonCommand(){
    return Commands.runOnce(    ()->agitate(Constants.HopperConstants.agitateSpeed),this);
  }
  public Command agitateoffCommand(){
    return Commands.runOnce(    ()->stopAgitate(),this);
  }

  //checking if there's NO fuel aka if there was no !, it would return true when there is not any fuel in hopper
  public boolean getNoFuel() {
    return m_BREAK.get();
  }
  
  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Kicker Sensor", m_BREAK.get());
    // This method will be called once per scheduler run
  }
  
}
