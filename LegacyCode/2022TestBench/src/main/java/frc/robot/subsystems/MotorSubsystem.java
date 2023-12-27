// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
public class MotorSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  private CANSparkMax m_Motor;
  public MotorSubsystem(int motor,boolean brushless) {
    if(brushless){
      m_Motor=new CANSparkMax(motor,CANSparkMaxLowLevel.MotorType.kBrushless);
    }
    else {
      m_Motor=new CANSparkMax(motor,CANSparkMaxLowLevel.MotorType.kBrushed);
    }
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public void setSpeed(double speed)
  {
    System.out.println("Speeds " + speed );
   
     m_Motor.set(speed);  
  }
  
  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
