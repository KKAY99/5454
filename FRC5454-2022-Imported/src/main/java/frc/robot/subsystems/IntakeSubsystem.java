// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
  VictorSPX m_IntakeMotor;

  /** Creates a new ExampleSubsystem. */
  public IntakeSubsystem(Integer IntakeMotorPort) {
    m_IntakeMotor= new VictorSPX(Constants.IntakePort);
  }
  public void runIntake(double power) {
    m_IntakeMotor.set(VictorSPXControlMode.PercentOutput,power);
    
  }

  public void stopIntake() {
    m_IntakeMotor.set(VictorSPXControlMode.PercentOutput,0);
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
