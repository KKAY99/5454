// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;

public class MotorSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  private CANSparkMax m_Motor
  public MotorSubsystem(int motor) {}
    m_motor=new CANSparkMax(motor);
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public void setSpeed(double speed)
  {
     m_Motor.set(speed);  
  }
  
  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
