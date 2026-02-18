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
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.geometry.Rotation2d;
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

public class TurretSubsystemOverlap extends SubsystemBase {
  private TalonFX m_turretMotor;
  private CANcoder m_encoder1;
  private CANcoder m_encoder2;
  private static final double GEAR_0_TOOTH_COUNT = 80.0;
  private static final double GEAR_1_TOOTH_COUNT = 10.0;
  private static final double GEAR_2_TOOTH_COUNT = 41.0;
  private static final double GEAR_3_TOOTH_COUNT = 40.0;

    private static final double SLOPE = (GEAR_2_TOOTH_COUNT * GEAR_1_TOOTH_COUNT)
            / ((GEAR_1_TOOTH_COUNT - GEAR_2_TOOTH_COUNT) * GEAR_0_TOOTH_COUNT);


  //private SparkAbsoluteEncoder m_encoder;

    public TurretSubsystemOverlap(int CanId1) {
    m_turretMotor = new TalonFX(CanId1);
    }
  public TurretSubsystemOverlap(int CanId1, int encoder1ID, int encoder2ID) {
    m_turretMotor = new TalonFX(CanId1);
    m_encoder1 = new CANcoder(encoder1ID);
    

    m_encoder2 = new CANcoder(encoder2ID);
 

    CANcoderConfiguration encoder1Config = new CANcoderConfiguration();
    encoder1Config.MagnetSensor.withAbsoluteSensorDiscontinuityPoint(1.0);
    encoder1Config.MagnetSensor.withMagnetOffset(-0.3271484375);
    encoder1Config.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;

    m_encoder1.getConfigurator().apply(encoder1Config);

    CANcoderConfiguration encoder2Config = new CANcoderConfiguration();
    encoder2Config.MagnetSensor.withAbsoluteSensorDiscontinuityPoint(1.0);
    encoder2Config.MagnetSensor.withMagnetOffset(-0.380615234);
    encoder2Config.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;

    m_encoder2.getConfigurator().apply(encoder2Config); 

  }
 
  public static double convertToClosestBoundedTurretAngleDegrees(
            double targetAngleDegrees, Rotation2d current, double forwardLimitDegrees, double reverseLimitDegrees) {
        double currentTotalRadians = (current.getRotations() * 2 * Math.PI);
        double closestOffset = Units.degreesToRadians(targetAngleDegrees) - current.getRadians();
        if (closestOffset > Math.PI) {

            closestOffset -= 2 * Math.PI;

        } else if (closestOffset < -Math.PI) {
            closestOffset += 2 * Math.PI;
        }

        double finalOffset = currentTotalRadians + closestOffset;
        if ((currentTotalRadians + closestOffset) % (2 * Math.PI)
                == (currentTotalRadians - closestOffset)
                        % (2 * Math.PI)) { // If the offset can go either way, go closer to zero
            if (finalOffset > 0) {
                finalOffset = currentTotalRadians - Math.abs(closestOffset);
            } else {
                finalOffset = currentTotalRadians + Math.abs(closestOffset);
            }
        }
        if (finalOffset > Units.degreesToRadians(forwardLimitDegrees)) { // if past upper rotation limit
            finalOffset -= (2 * Math.PI);
        } else if (finalOffset < Units.degreesToRadians(reverseLimitDegrees)) { // if below lower rotation limit
            finalOffset += (2 * Math.PI);
        }

        return Units.radiansToDegrees(finalOffset);
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
     
        SmartDashboard.putNumber("difference",difference);
        SmartDashboard.putNumber("e1Rotations",e1Rotations);
        SmartDashboard.putNumber("E1RotationsFloored",e1RotationsFloored);
        if (turretAngle - difference < -100) {
            turretAngle += GEAR_1_TOOTH_COUNT / GEAR_0_TOOTH_COUNT * 360.0;
        } else if (turretAngle - difference > 100) {
            turretAngle -= GEAR_1_TOOTH_COUNT / GEAR_0_TOOTH_COUNT * 360.0;
        }
        return turretAngle;
    }
 
  private Supplier<Angle> getAngle(CANcoder encoder){
    return () -> encoder.getAbsolutePosition().getValue();
  }
  
  public void showEncoderPositions(){
        SmartDashboard.putNumber("Turret Motor" , m_turretMotor.getPosition().getValueAsDouble());        
        SmartDashboard.putNumber("Encoder 1",m_encoder1.getAbsolutePosition().getValueAsDouble());
       
        SmartDashboard.putNumber("Encoder 1 ABS",m_encoder1.getAbsolutePosition().getValueAsDouble());
        SmartDashboard.putNumber("Encoder 1 REL",m_encoder1.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("Encoder 2",m_encoder2.getAbsolutePosition().getValueAsDouble());
        double e1Degrees = m_encoder1.getAbsolutePosition().getValueAsDouble()*360;
        double e2Degrees = m_encoder2.getAbsolutePosition().getValueAsDouble()*360;
        SmartDashboard.putNumber("Encoder 1 ANG",e1Degrees);
        SmartDashboard.putNumber("Encoder 2 ANG",e2Degrees);
        double angle=calculateTurretAngleFromCANCoderDegrees(e1Degrees,e2Degrees);   
        SmartDashboard.putNumber("Turret Angle:",angle);
      }
  public void moveTurret(double speed) 
  {
      showEncoderPositions();
      
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