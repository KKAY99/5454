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

public class TurretSubsystem extends SubsystemBase {
  CANSparkMax m_TurretMotor;

  /** Creates a new ExampleSubsystem. */
  public TurretSubsystem(Integer turretMotorPort) {    
       m_TurretMotor = new CANSparkMax(turretMotorPort, MotorType.kBrushless);  
       m_TurretMotor.setInverted(false);
       m_TurretMotor.setOpenLoopRampRate(0.25);
  }
  public void turn(double power) {
    m_TurretMotor.set(power);
  }

  public void stop() {
    m_TurretMotor.set(0);
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
