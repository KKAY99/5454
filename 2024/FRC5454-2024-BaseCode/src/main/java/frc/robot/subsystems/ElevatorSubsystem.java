// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

 
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj.DigitalInput;

public class ElevatorSubsystem extends SubsystemBase {
  CANSparkMax m_Motor;
  RelativeEncoder m_elevatorEncoder;
  SparkMaxPIDController m_pidController;
  DigitalInput m_limit;
  private double m_targetPos;
  private boolean m_homed = false;

  /** Creates a new ExampleSubsystem. */
  public ElevatorSubsystem(Integer MotorPort, Integer limitswitchport) {
    m_Motor = new CANSparkMax(MotorPort, MotorType.kBrushless);   
    
    m_Motor.setOpenLoopRampRate(0.25);
    m_Motor.setSmartCurrentLimit(30);  // likely gets ignored due to brushed motor
    m_Motor.setSecondaryCurrentLimit(30); //Set as well at 30
    m_Motor.setIdleMode(CANSparkMax.IdleMode.kBrake);
    m_elevatorEncoder=m_Motor.getEncoder();
    m_limit = new DigitalInput(limitswitchport);
    
    m_pidController = m_Motor.getPIDController();
    m_pidController.setFeedbackDevice(m_elevatorEncoder);   
    m_pidController.setP(Constants.Lift.liftKP);
    m_pidController.setI(Constants.Lift.liftKI);
    m_pidController.setD(Constants.Lift.liftKD);
    m_pidController.setIZone(Constants.Lift.liftKIZ);
    m_pidController.setFF(Constants.Lift.liftKFF);
    m_pidController.setOutputRange(Constants.Lift.minOutPut, Constants.Lift.maxOutput);
   
  }

  public void runWithOutLimit(double power){
    moveElevator(power, false);
  }

  public void runWithLimit(double power){
    moveElevator(power, true);
  }

  public void moveElevator(double power, boolean checklimit) {
    double speed = power;
   // System.out.println("Power" + power);
    if(checklimit && hasHitPhysicalLimitSwitch() && power > 0){
      speed = 0;
      System.out.println("Lower Limit Switch-Elevator");
    }

    if(checklimit && hasHitMaxLimit() && power < 0){
      speed = 0;
      System.out.println("Higher Limit Switch-Elevator");
    }
    
    m_pidController.setReference(speed, ControlType.kDutyCycle);
    //m_Motor.set(speed);
  }

  public void setHomed(boolean value){
    m_homed=value;
  }
  public boolean hasHomed(){
    return m_homed;
  }

  public void stop() {
    m_Motor.set(0);
  }

  public boolean hasHitPhysicalLimitSwitch(){
    boolean hasLimitBeenHit = m_limit.get();
    boolean returnValue = false;

    if(hasLimitBeenHit){
      returnValue = false;
    }else{
      returnValue = true;
    }

    return returnValue;
  }

  public boolean hasHitMaxLimit(){
    if(getElevatorPos() <= Constants.Elevator.maxLimit){
      return true;
    }else{
      return false;
    }
  }

  public void setZero(){
    m_elevatorEncoder.setPosition(0);
  }

  public void SetPosAndMove(double targetPos){
    m_pidController.setReference(targetPos, CANSparkMax.ControlType.kPosition);
    m_targetPos = targetPos;
   }

  public double getElevatorPos(){
    try {
      return m_elevatorEncoder.getPosition();
    }
    catch (Exception e){
      System.out.println("Pneumatics Failure");
      System.out.println("Exception Message: " + e.getMessage());
      System.out.println("StackTrace:" + e.getStackTrace().toString());
      return 0;  
    }
  }

  public void disableElevatorBrakeMode(){
    m_Motor.setIdleMode(IdleMode.kCoast);
  }

  public void resetElevatorBrakeModeToNormal(){
    m_Motor.setIdleMode(IdleMode.kBrake);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //System.out.println("ELS" + m_limit.get());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
