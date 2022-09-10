// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxRelativeEncoder.Type;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.DigitalInput;
//import com.revrobotics.SparkMaxAlternateEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;

public class TurretSubsystem extends SubsystemBase {

  CANSparkMax m_turretMotor;
  RelativeEncoder m_turretEncoder;
  //private static final SparkMaxAlternateEncoder.Type kAltEncType = SparkMaxAlternateEncoder.Type.kQuadrature;
  private static final int kCPR = 8192;

  private DigitalInput m_limitRightSwitch;
  private boolean m_encoderHasHomed = false;
  private double m_safePositionforClimb;
  private double m_turretSafeMoveSpeed;
  private boolean m_turretLockedMode = false;

  /** Creates a new ExampleSubsystem. */
  public TurretSubsystem(Integer turretMotorPort, int rightSwitch, double safePositionforClimb,
      double safetyMoveSpeed) {

    m_turretMotor = new CANSparkMax(turretMotorPort, MotorType.kBrushless);

    m_turretMotor.setIdleMode(IdleMode.kBrake);
    //m_turretEncoder = m_turretMotor.getEncoder(Type.kQuadrature, kCPR);
    m_turretEncoder=m_turretMotor.getEncoder(Type.kHallSensor,42);
    //m_turretMotor.setInverted(false);
    m_turretMotor.setInverted(true);
    m_limitRightSwitch = new DigitalInput(rightSwitch);
    m_safePositionforClimb = safePositionforClimb;
    m_turretSafeMoveSpeed = safetyMoveSpeed;
  }

  public void turn(double power) {
    System.out.println("Turn- Power is " + power);
    m_turretMotor.set(power);
  }

  public void setEncoderPosition(double position) {
    m_turretEncoder.setPosition(position);
  }

  public void stop() {

    System.out.println("stopping turret");
    m_turretMotor.set(0);
  }

  public boolean isMovingLeft(double targetspeed) {
    return m_turretEncoder.getVelocity() < 0 || targetspeed < 0;
  }

  public boolean isMovingRight(double targetspeed) {
    return m_turretEncoder.getVelocity() > 0 || targetspeed > 0;
 
  }

  public boolean isClearofClimber() {
    if (m_encoderHasHomed) {
      double currentPos = Math.abs(getPosition());
      return (currentPos >= Math.abs(m_safePositionforClimb));
    } else {
      return false; // if encoder has not homed we can not check for falcon safety
    }
  }

  public void movePastSafetyPosition() {
    turn(m_turretSafeMoveSpeed);
  }

  public double getPosition() {
    if(m_turretMotor.getInverted() == false){
      return m_turretEncoder.getPosition(); 
    }else{
      //if inverted reverse turret pos value
      return -m_turretEncoder.getPosition();
    }
    
  }

  public boolean hitLeftLimit() {
    boolean returnValue = false;
    if (m_encoderHasHomed) {
      // System.out.println("Checking Left - " + m_limitLeftSwitch.get() + "-" +
      // m_turretEncoder.getPosition());
      returnValue =
          (getPosition() < Constants.LimitSwitches.TurretLeftEncoder);
    } else {
      returnValue = true; }
    // System.out.println(isMovingLeft());
    // System.out.println("turret Left Check " + encoderHasHomed + " - " +
    // m_turretEncoder.getPosition() + " - " + returnValue);
    return returnValue;
  }

  public boolean hitRightPhysicalLimit() {
    return (m_limitRightSwitch.get());
  }

  public void setLocked() {
    m_turretLockedMode = true;
  }

  public void overlockTuretLock() {
    m_turretLockedMode = false;
  }

  public boolean isLocked() {
    return m_turretLockedMode;
  }

  public boolean hitRightLimit() {
    if (m_encoderHasHomed) {
      // System.out.println("Checking Right - " + m_limitLeftSwitch.get() + "-" +
      // m_turretEncoder.getPosition());

      return (m_limitRightSwitch.get() ||
          (getPosition()> Constants.LimitSwitches.TurretRightEncoder));
    } else {
      return (m_limitRightSwitch.get());
    }
  }

  public boolean hasHomed() {
    return m_encoderHasHomed;
  }

  public void setHomeforTurret() {
    m_turretEncoder.setPosition(0);
    m_encoderHasHomed = true;
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
