// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utilities.ObsidianCANSparkMax;
import edu.wpi.first.wpilibj.Servo;


public class WinchSubsystem extends SubsystemBase {
  private ObsidianCANSparkMax m_winchMotor;
  private Servo m_servo;


  public WinchSubsystem(int canID, int servoID) {
    m_winchMotor = new ObsidianCANSparkMax(canID, MotorType.kBrushless, true, Constants.k80Amp);
    m_servo = new Servo(servoID);

  }

  public boolean checkCANConnections(){
    boolean returnValue=true;
    double var=0;

    try{
      var=m_winchMotor.getDeviceId();
      var=m_servo.get();
    }catch(Exception e){
      returnValue=false;
    }

    return returnValue;
  }

  public void engageServo(){
    m_servo.setAngle(100); // pos is most likely incorrect old code
  }

  public void disengageServo(){
    m_servo.setAngle(80); // pos is most likely incorrect old code
  }

  public void run(double speed){
    m_winchMotor.set(speed);
  }

  public void stop(){
    m_winchMotor.stopMotor();
  }



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
