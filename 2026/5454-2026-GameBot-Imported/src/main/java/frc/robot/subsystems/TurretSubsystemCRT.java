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

public class TurretSubsystemCRT extends SubsystemBase {
  private TalonFX m_turretMotor;
  private CANcoder m_encoder1;
  private CANcoder m_encoder2;
  private EasyCRT m_EasyCRT;
  private EasyCRTConfig m_CRTConfig;
  private static final int GEAR_0_TOOTH_COUNT = 80;
  private static final int GEAR_1_TOOTH_COUNT = 10;
  private static final int GEAR_2_TOOTH_COUNT = 41;
  private static final int GEAR_3_TOOTH_COUNT = 40;

    private static final double SLOPE = (GEAR_2_TOOTH_COUNT * GEAR_1_TOOTH_COUNT)
            / ((GEAR_1_TOOTH_COUNT - GEAR_2_TOOTH_COUNT) * GEAR_0_TOOTH_COUNT);

 //11 is on 10tgear
  //private SparkAbsoluteEncoder m_encoder;

    public TurretSubsystemCRT(int CanId1) {
    m_turretMotor = new TalonFX(CanId1);
    }
  public TurretSubsystemCRT(int CanId1, int encoder1ID, int encoder2ID) {
    m_turretMotor = new TalonFX(CanId1);
    m_encoder1 = new CANcoder(encoder1ID);
    

    m_encoder2 = new CANcoder(encoder2ID);
 
 m_CRTConfig =
        new EasyCRTConfig(
                () -> Rotations.of(m_encoder1.getAbsolutePosition().getValueAsDouble()),
                () -> Rotations.of(m_encoder2.getAbsolutePosition().getValueAsDouble()))
            /* .withAbsoluteEncoder1Gearing(
                GEAR_0_TOOTH_COUNT,
                GEAR_1_TOOTH_COUNT)
               .withAbsoluteEncoder2Gearing
              ( GEAR_0_TOOTH_COUNT,
                GEAR_1_TOOTH_COUNT,
                GEAR_2_TOOTH_COUNT,
                GEAR_3_TOOTH_COUNT)
              */
                .withEncoderRatios(8,8.2) 
                //.withEncoderRatios(8,7.8); 
                .withAbsoluteEncoderOffsets(Rotations.of(0.455),Rotations.of(-0.375))
                .withMechanismRange(
                Rotations.of(TurretConstants.MIN_ROT_DEG / 360),
                Rotations.of(TurretConstants.MAX_ROT_DEG / 360))
          
            .withMatchTolerance(Rotations.of(0.1)
        );   
    CANcoderConfiguration encoder1Config = new CANcoderConfiguration();
    encoder1Config.MagnetSensor.withAbsoluteSensorDiscontinuityPoint(1.0);
    //encoder1Config.MagnetSensor.withMagnetOffset(-0.455);
    encoder1Config.MagnetSensor.SensorDirection=SensorDirectionValue.CounterClockwise_Positive;
    m_encoder1.getConfigurator().apply(encoder1Config);

    CANcoderConfiguration encoder2Config = new CANcoderConfiguration();
    encoder2Config.MagnetSensor.withAbsoluteSensorDiscontinuityPoint(1.0);
    //encoder2Config.MagnetSensor.withMagnetOffset(-0.325);
    
    encoder2Config.MagnetSensor.SensorDirection=SensorDirectionValue.Clockwise_Positive;
    m_encoder2.getConfigurator().apply(encoder2Config); 

    m_EasyCRT = new EasyCRT(m_CRTConfig);
    SmartDashboard.putNumber("UniqueCoverage",m_CRTConfig.getUniqueCoverage().orElse(Degrees.of(0.0)).in(Degrees));
    SmartDashboard.putBoolean("SatisfiesRange",m_CRTConfig.coverageSatisfiesRange());
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
       SmartDashboard.putNumber("EasyCRT Enc 1", m_CRTConfig.getAbsoluteEncoder1Angle().in(Degrees));
    SmartDashboard.putNumber(
        "EasyCRT Enc 1 Ratio", m_CRTConfig.getEncoder1RotationsPerMechanismRotation());
    SmartDashboard.putNumber("EasyCRT Enc 2", m_CRTConfig.getAbsoluteEncoder2Angle().in(Degrees));
    SmartDashboard.putNumber(
        "EasyCRT Enc 2 Ratio", m_CRTConfig.getEncoder2RotationsPerMechanismRotation());
        SmartDashboard.putString("Last Status",m_EasyCRT.getLastStatus().toString());
        m_EasyCRT
        .getAngleOptional()
        .ifPresent(angle -> SmartDashboard.putNumber("Turret Angle", angle.in(Degrees)));
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