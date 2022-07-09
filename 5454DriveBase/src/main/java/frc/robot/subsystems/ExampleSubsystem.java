// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
public class ExampleSubsystem extends SubsystemBase {
  private VictorSPX m_TalonFx10;
  private VictorSPX m_TalonFx11;
  private VictorSPX m_TalonFx12;
  private VictorSPX m_TalonFx13;
  /** Creates a new ExampleSubsystem. */
  public ExampleSubsystem() {
    m_TalonFx10 = new VictorSPX(10);
    m_TalonFx11 = new VictorSPX(11);
    m_TalonFx12 = new VictorSPX(12);
    m_TalonFx13 = new VictorSPX(13);
  }
 
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public void setSpeed(double leftspeed, double rightspeed){
    System.out.println("Setting Speed " + leftspeed);
    m_TalonFx10.set(VictorSPXControlMode.PercentOutput,leftspeed);
    m_TalonFx11.set(VictorSPXControlMode.PercentOutput,leftspeed);
    m_TalonFx12.set(VictorSPXControlMode.PercentOutput,rightspeed);
    m_TalonFx13.set(VictorSPXControlMode.PercentOutput,rightspeed);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
