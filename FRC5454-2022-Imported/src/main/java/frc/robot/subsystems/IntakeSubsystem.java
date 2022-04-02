// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

 
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
  CANSparkMax m_IntakeMotor;

  /** Creates a new ExampleSubsystem. */
  public IntakeSubsystem(Integer IntakeMotorPort) {
    m_IntakeMotor = new CANSparkMax(IntakeMotorPort, MotorType.kBrushed);   
    m_IntakeMotor.setOpenLoopRampRate(0.25);
    m_IntakeMotor.setSmartCurrentLimit(30);  // likely gets ignored due to brushed motor
    m_IntakeMotor.setSecondaryCurrentLimit(30); //Set as well at 30
  }
  public void runIntake(double power) {
    m_IntakeMotor.set(power);
    
  }

  public void stopIntake() {
    m_IntakeMotor.set(0);
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
