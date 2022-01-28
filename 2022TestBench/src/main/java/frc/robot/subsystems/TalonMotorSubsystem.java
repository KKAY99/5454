// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.ControlMode;
//import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
public class TalonMotorSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  private WPI_TalonFX m_Motor;
  public TalonMotorSubsystem(int motor,boolean brushless) {
    if(brushless){
      m_Motor=new WPI_TalonFX(motor);
     
    }
    else {
      m_Motor=new WPI_TalonFX(motor);
      
    }
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public void setSpeed(double speed)
  {
    System.out.println("Speeds " + speed );
   
     m_Motor.set(ControlMode.PercentOutput,speed);  
  }
  
  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
