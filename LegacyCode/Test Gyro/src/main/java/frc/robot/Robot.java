// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.SPI;
/**
 * This is a sample program to demonstrate how to use a gyro sensor to make a robot drive straight.
 * This program uses a joystick to drive forwards and backwards while the gyro is used for direction
 * keeping.
 */
public class Robot extends TimedRobot {
 
   AHRS gyro;

  @Override
  public void robotInit() {
   

  gyro = new AHRS(SPI.Port.kMXP);
  gyro.enableLogging(true);
  }

  /**
   * The motor speed is set from the joystick while the DifferentialDrive turning value is assigned
   * from the error between the setpoint and the gyro angle.
   */

   
  @Override
  public void teleopPeriodic() {
    


  SmartDashboard.putNumber("IMU_Yaw", gyro.getYaw());
  SmartDashboard.putNumber("IMU_Pitch", gyro.getPitch());
  SmartDashboard.putNumber("IMU_Roll", gyro.getRoll());
}
}