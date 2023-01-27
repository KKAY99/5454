
/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.*;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {
  /**
   * Creates a new ShooterSubsystem
   */
  TalonSRX m_topTalon = new TalonSRX (ShooterConstants.shooterTopPort);
  TalonSRX m_bottomTalon = new TalonSRX(ShooterConstants.shooterBottomPort);
  public ShooterSubsystem() {

   
    m_topTalon.setNeutralMode(NeutralMode.Coast);
    m_bottomTalon.setNeutralMode(NeutralMode.Coast);
  }

  public void setSpeed(double speed){
    System.out.print("Setting Shooter Speed - " + speed);
  
    m_topTalon.set(ControlMode.PercentOutput,0-speed);
    m_bottomTalon.set(ControlMode.PercentOutput,speed);
    
    
  } 

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
