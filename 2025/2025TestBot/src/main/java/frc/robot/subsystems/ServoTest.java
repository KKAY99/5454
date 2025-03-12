// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Servo;

public class ServoTest extends SubsystemBase {
  private Servo m_Servo;
  
  public ServoTest(int servoPMW) {
    m_Servo = new Servo(servoPMW);
  }

  public void move180(){
    m_Servo.setAngle(80); //mininimum num is 180
    System.out.println(m_Servo.getAngle());
  }

  public void moveneg180(){
    m_Servo.setAngle(100); //minimum num is 0
    System.out.println(m_Servo.getAngle());
  }

  @Override
  public void periodic() {
  }
}
