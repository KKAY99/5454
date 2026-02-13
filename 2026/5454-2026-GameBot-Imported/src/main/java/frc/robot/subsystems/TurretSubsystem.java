package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utilities.ObsidianCANSparkMax;
import com.ctre.phoenix6.hardware.CANcoder;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.TurretConstants;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import yams.mechanisms.*;
import yams.units.EasyCRT;
import yams.units.EasyCRTConfig;
import java.util.function.Supplier;
import edu.wpi.first.units.measure.Angle;
import static edu.wpi.first.units.Units.Rotation;
import static edu.wpi.first.units.Units.Rotations;
import com.ctre.phoenix6.hardware.TalonFX;

public class TurretSubsystem extends SubsystemBase {
  private TalonFX m_turretMotor;
  private CANcoder m_encoder1;
  private CANcoder m_encoder2;
  private EasyCRT m_EasyCRT;
  private final int kTurretTeeth=100;
  private final int kEncoder1Teeth=41;
  private final int kEncoder2Teeth=42;


  //private SparkAbsoluteEncoder m_encoder;

    public TurretSubsystem(int CanId1) {
    m_turretMotor = new TalonFX(CanId1);
    }
  public TurretSubsystem(int CanId1, int encoder1ID, int encoder2ID) {
    m_turretMotor = new TalonFX(CanId1);
    m_encoder1 = new CANcoder(encoder1ID);
    
    m_encoder2 = new CANcoder(encoder2ID);
    

     EasyCRTConfig easyCRTConfig =
        new EasyCRTConfig(
                () -> Rotations.of(m_encoder1.getAbsolutePosition().getValueAsDouble()),
                () -> Rotations.of(m_encoder2.getAbsolutePosition().getValueAsDouble()))
            .withAbsoluteEncoder1Gearing(
                TurretConstants.GEAR_0_TOOTH_COUNT, TurretConstants.GEAR_1_TOOTH_COUNT)
            .withAbsoluteEncoder2Gearing(
                TurretConstants.GEAR_0_TOOTH_COUNT,
                TurretConstants.GEAR_1_TOOTH_COUNT,
                TurretConstants.GEAR_2_TOOTH_COUNT)
            .withMechanismRange(
                Rotations.of(TurretConstants.MIN_ROT_DEG / 360),
                Rotations.of(TurretConstants.MAX_ROT_DEG / 360))
            .withMatchTolerance(Rotations.of(TurretConstants.CRT_MATCH_TOLERANCE));

    m_EasyCRT = new EasyCRT(easyCRTConfig);
  }

 /*  private void setEncoderConfig(CANcoder canCoder{
    CANcoderConfiguration m_config = new CANcoderConfiguration();
    m_config.MagnetSensor.
  })*/

  private Supplier<Angle> getAngle(CANcoder encoder){
    return () -> encoder.getAbsolutePosition().getValue();
  }
  

  public void moveTurret(double speed) 
  {
      System.out.println("Encoder 1 pos:" + m_encoder1.getAbsolutePosition());
      System.out.println("Encoder 2 pos:" + m_encoder2.getAbsolutePosition());
      System.out.println("Last Status:" + m_EasyCRT.getLastStatus());
      m_EasyCRT
        .getAngleOptional()
        .ifPresent(angle -> SmartDashboard.putNumber("Turret Angle", angle.in(Degrees)));

      System.out.println("Turret Move:" + speed);
    m_turretMotor.set(speed);
  }

  public void stopTurret(){
    m_turretMotor.stopMotor();
  }
  public Command turretManualCommand(){
      return Commands.startEnd(    ()->moveTurret(TurretConstants.turretSpeed),
                                          ()->stopTurret(),
                                          this);
  }

  public Command turretMoveManualCommand(){
    return Commands.runOnce(    ()->moveTurret(TurretConstants.turretSpeed),this);
  }
  public Command turretStopManualCommand(){
    return Commands.runOnce(    ()->stopTurret(),this);
  }
}