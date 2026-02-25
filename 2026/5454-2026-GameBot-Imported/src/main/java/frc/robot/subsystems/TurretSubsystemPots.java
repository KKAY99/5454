package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
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
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
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
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

public class TurretSubsystemPots extends SubsystemBase {
  private TalonFX m_turretMotor;
  private CANcoder m_encoder1;
  private CANcoder m_encoder2;
  private AnalogPotentiometer m_POTS;
  private final double kPotsLowLimit=0.30;
  private final double kPotsHighLimit=0.70;
  private final double kGearReduction=10;  
  public TurretSubsystemPots(int CanId1, int encoder1ID, int encoder2ID,int potsPort) {
    m_POTS = new AnalogPotentiometer(potsPort,1,0); 
    m_turretMotor = new TalonFX(CanId1);
    TalonFXConfiguration motorConfig = new TalonFXConfiguration();
    motorConfig.MotorOutput.NeutralMode=NeutralModeValue.Brake;
    m_turretMotor.getConfigurator().apply(motorConfig);
    m_encoder1 = new CANcoder(encoder1ID);
    

    m_encoder2 = new CANcoder(encoder2ID);
    
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
    }

   public void moveTurret(double speed) 
  {
      showEncoderPositions();
      if(atLimit(speed)){
        stopTurret();
        System.out.println("Turret Stopped / At Limit");
   
      } else {
          m_turretMotor.set(speed);
          System.out.println("Turret Move:" + speed);
   
      }

  }
  public boolean atLimit(double speed){
    boolean returnValue=false;
    System.out.println("Limit Check:" + speed + " -- " + m_POTS.get());
    if(speed<0 && m_POTS.get()<kPotsLowLimit){// Moving Left towards zero on POTS
        returnValue=true;
    }else if (speed>0 && m_POTS.get()>kPotsHighLimit){ //moving right towards One on POTS
        returnValue=true;
    } // Moving Left towards zero on POTS
    return returnValue;
  }
  
  public void stopTurret(){
    m_turretMotor.stopMotor();
  }
  public Command turretManualCommand(){
      return Commands.startEnd(    ()->moveTurret(TurretConstants.turretSpeed),
                                          ()->stopTurret(),
                                          this);
  }

  
  public void periodic(){
    SmartDashboard.putBoolean("At High Limit Limit",atLimit(1));
    SmartDashboard.putBoolean("At Low Limit Limit",atLimit(-1));
    SmartDashboard.putNumber("Motor Rotations",m_turretMotor.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("POTS",m_POTS.get());
    SmartDashboard.putNumber("POTS Angle",m_POTS.get()*3600/kGearReduction);
    //SmartDashboard.putBoolean("AtLimit",atLimit(m_speed));
    //SmartDashboard.putNumber("POTS",m_POTS.;
  }
}