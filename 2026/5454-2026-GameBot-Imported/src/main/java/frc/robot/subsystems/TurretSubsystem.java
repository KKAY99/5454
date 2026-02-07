package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utilities.ObsidianCANSparkMax;
import com.ctre.phoenix6.hardware.CANcoder;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import yams.units.EasyCRT;
import yams.units.EasyCRTConfig;
import java.util.function.Supplier;
import edu.wpi.first.units.measure.Angle;
import static edu.wpi.first.units.Units.Rotation;
import static edu.wpi.first.units.Units.Rotations;

public class TurretSubsystem extends SubsystemBase {
  private ObsidianCANSparkMax m_turretMotor;
  private CANcoder m_encoder1;
  private CANcoder m_encoder2;
  private EasyCRT m_EasyCRT;
  //private SparkAbsoluteEncoder m_encoder;

  public TurretSubsystem(int CanId1, int encoder1, int encoder2) {
    m_turretMotor = new ObsidianCANSparkMax(CanId1, MotorType.kBrushless, true);
    m_encoder1 = new CANcoder(encoder1);
    m_encoder2 = new CANcoder(encoder2);

    EasyCRTConfig easyCRTConfigCRTConfig=
         new EasyCRTConfig(getAngle(m_encoder1), getAngle(m_encoder2))
        .withCommonDriveGear(
            /* commonRatio (mech:drive) */ 12.0,
            /* driveGearTeeth */ 50,
            /* encoder1Pinion */ 19,
            /* encoder2Pinion */ 23)
        .withAbsoluteEncoderOffsets(Rotations.of(0.0), Rotations.of(0.0)) // set after mechanical zero
        .withMechanismRange(Rotations.of(-1.0), Rotations.of(2.0)) // -360 deg to +720 deg
        .withMatchTolerance(Rotations.of(0.06)) // ~1.08 deg at encoder2 for the example ratio
        .withAbsoluteEncoderInversions(false, false)
        .withCrtGearRecommendationConstraints(
            /* coverageMargin */ 1.2,
            /* minTeeth */ 15,
            /* maxTeeth */ 45,
            /* maxIterations */ 30);
    m_EasyCRT = new EasyCRT(easyCRTConfigCRTConfig);
  }

  private Supplier<Angle> getAngle(CANcoder encoder){
    return () -> encoder.getPosition().getValue();
  }

  public void moveTurret(double speed) {
    m_turretMotor.set(speed);
  }

  public void stopTurret(){
    m_turretMotor.stopMotor();
  }
/* 
  public Command intakeonCommand(){
    return Commands.runOnce(    ()->runIntake(IntakeConstants.highSpeed,IntakeConstants.lowSpeed),this);
  }
  public Command intakeoffCommand(){
    return Commands.runOnce(    ()->stopIntake(),this);
  }
*/
}