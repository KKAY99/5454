// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.revrobotics.CANSparkMax;
import com.revrobotics.EncoderType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxRelativeEncoder.Type;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Counter;
import com.revrobotics.SparkMaxAlternateEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
public class TurretSubsystem extends SubsystemBase {
  
  CANSparkMax m_turretMotor;
  RelativeEncoder m_turretEncoder;
  private static final SparkMaxAlternateEncoder.Type kAltEncType = SparkMaxAlternateEncoder.Type.kQuadrature;
  private static final int kCPR = 8192;

  private DigitalInput m_limitLeftSwitch;
  private DigitalInput m_limitRightSwitch;

  /** Creates a new ExampleSubsystem. */
  public TurretSubsystem(Integer turretMotorPort,int leftSwitch, int rightSwitch) {    
       m_turretMotor = new CANSparkMax(turretMotorPort, MotorType.kBrushed);  
       m_turretMotor.setIdleMode(IdleMode.kBrake);
       //m_turretEncoder = m_turretMotor.getAlternateEncoder(klAtEncType,kCPR);
       m_turretEncoder = m_turretMotor.getEncoder(Type.kQuadrature,kCPR);
       m_turretMotor.setInverted(false);
       m_limitRightSwitch=new DigitalInput(rightSwitch);
       m_limitLeftSwitch=new DigitalInput(leftSwitch);
      
  
      }
  public void turn(double power) {
    System.out.println("setting turret power "+ power);
    m_turretMotor.set(power);
  }
  public void setEncoderPosition(double position){
    m_turretEncoder.setPosition(position);
  }
  public void stop() {
    System.out.println("stopping turret");

    m_turretMotor.set(0);
  }
  public boolean isMovingLeft(){
    return m_turretEncoder.getVelocity() <0;
 }
 public boolean isMovingRight(){
     return m_turretEncoder.getVelocity()>0;
 }


  public double getPosition(){
    return m_turretEncoder.getPosition();
     }
  public boolean hitLeftLimit()
  {  
    return (m_limitLeftSwitch.get());
  }

  public boolean hitRightLimit()
  {
    return (m_limitRightSwitch.get());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
