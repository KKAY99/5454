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
import com.ctre.phoenix6.signals.SensorDirectionValue;

public class TurretSubsystem extends SubsystemBase {
  private TalonFX m_turretMotor;
  private CANcoder m_encoder1;
  private CANcoder m_encoder2;
  private EasyCRT m_EasyCRT;
  private static final double GEAR_0_TOOTH_COUNT = 80.0;
  private static final double GEAR_1_TOOTH_COUNT = 41.0;
  private static final double GEAR_2_TOOTH_COUNT = 40.0;

    private static final double SLOPE = (GEAR_2_TOOTH_COUNT * GEAR_1_TOOTH_COUNT)
            / ((GEAR_1_TOOTH_COUNT - GEAR_2_TOOTH_COUNT) * GEAR_0_TOOTH_COUNT);


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
            .withAbsoluteEncoder1Inverted(false)
            .withAbsoluteEncoder2Inverted(false)
            .withMatchTolerance(Rotations.of(TurretConstants.CRT_MATCH_TOLERANCE));

    CANcoderConfiguration encoder1Config = new CANcoderConfiguration();
    encoder1Config.MagnetSensor.withAbsoluteSensorDiscontinuityPoint(1.0);
    encoder1Config.MagnetSensor.withMagnetOffset(-0.455);
    m_encoder1.getConfigurator().apply(encoder1Config);

    CANcoderConfiguration encoder2Config = new CANcoderConfiguration();
    encoder2Config.MagnetSensor.withAbsoluteSensorDiscontinuityPoint(1.0);
    encoder2Config.MagnetSensor.withMagnetOffset(-0.325);
    m_encoder2.getConfigurator().apply(encoder2Config); 

    m_EasyCRT = new EasyCRT(easyCRTConfig);
  }

 /*  private void setEncoderConfig(CANcoder canCoder{
    CANcoderConfiguration m_config = new CANcoderConfiguration();
    m_config.MagnetSensor.
  })*/

  private Supplier<Angle> getAngle(CANcoder encoder){
    return () -> encoder.getAbsolutePosition().getValue();
  }
  
  public void showEncoderPositions(){
        SmartDashboard.putNumber("Encoder 1",m_encoder1.getAbsolutePosition().getValueAsDouble());
        SmartDashboard.putNumber("Encoder 2",m_encoder2.getAbsolutePosition().getValueAsDouble());
        m_EasyCRT
        .getAngleOptional()
        .ifPresent(angle -> SmartDashboard.putNumber("Turret Angle", angle.in(Degrees)));
   }
 public static double calculateTurretAngleFromCANCoderDegrees(double e1, double e2) {
        double difference = e2 - e1;
        if (difference > 250) {
            difference -= 360;
        }
        if (difference < -250) {
            difference += 360;
        }
        difference *= SLOPE;

        double e1Rotations = (difference * GEAR_0_TOOTH_COUNT / GEAR_1_TOOTH_COUNT) / 360.0;
        double e1RotationsFloored = Math.floor(e1Rotations);
        double turretAngle = (e1RotationsFloored * 360.0 + e1) * (GEAR_1_TOOTH_COUNT / GEAR_0_TOOTH_COUNT);
        if (turretAngle - difference < -100) {
            turretAngle += GEAR_1_TOOTH_COUNT / GEAR_0_TOOTH_COUNT * 360.0;
        } else if (turretAngle - difference > 100) {
            turretAngle -= GEAR_1_TOOTH_COUNT / GEAR_0_TOOTH_COUNT * 360.0;
        }
        return turretAngle;
    }
  public void moveTurret(double speed) 
  {
      showEncoderPositions();
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