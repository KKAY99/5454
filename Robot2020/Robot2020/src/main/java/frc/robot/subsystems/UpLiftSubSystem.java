
/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.*;
import frc.robot.Constants.LiftConstants;

public class UpLiftSubSystem extends SubsystemBase {
  /**
   * Creates a new IntakeSubsystem.
   */
  VictorSPX m_victor = new VictorSPX(LiftConstants.upLift);
  public UpLiftSubSystem() {
   
  }

  public void setSpeed(double speed){
    m_victor.set(ControlMode.PercentOutput,0-speed);
    System.out.print("Setting Victor Speed - " + speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
