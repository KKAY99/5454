// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import frc.robot.Constants.HoodConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.utilities.ObsidianCANSparkMax;
import edu.wpi.first.networktables.NetworkTableInstance.NetworkMode;
import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class NewShooterSubsystem extends SubsystemBase {
    private TalonFX m_1shooterMotor;
    private TalonFX m_2shooterMotor;
    private TalonFX m_hoodMotor;
    private CANcoder m_hoodCoder = new CANcoder(Constants.HoodConstants.hoodCoderCANID);
  
    //private TalonFX m_kickerMotor;
    private ObsidianCANSparkMax m_kickerMotor;
  public NewShooterSubsystem(int shooter1CANID, int shooter2CANID, int kickerCANID,int hoodCANID) {
    m_1shooterMotor = new TalonFX(shooter1CANID);
    configureShootermotor(m_1shooterMotor);
    m_1shooterMotor.setNeutralMode(NeutralModeValue.Coast);
    m_2shooterMotor = new TalonFX(shooter2CANID);
    configureShootermotor(m_2shooterMotor);
    m_2shooterMotor.setNeutralMode(NeutralModeValue.Coast);
    m_hoodMotor = new TalonFX(hoodCANID);
    m_kickerMotor = new ObsidianCANSparkMax(kickerCANID, MotorType.kBrushless,false,Constants.k70Amp);
    
        /* Configure CANcoder to zero the magnet appropriately */
    CANcoderConfiguration hoodCoder_cfg = new CANcoderConfiguration();
    hoodCoder_cfg.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1.0; // unsigned [0,1) range, so 1.0 is the same as 0.0, which means the discontinuity is at the wrap-around point
    hoodCoder_cfg.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
    hoodCoder_cfg.MagnetSensor.withMagnetOffset(Rotations.of(Constants.HoodConstants.hoodOffset));
    m_hoodCoder.getConfigurator().apply(hoodCoder_cfg);
    TalonFXConfiguration hoodMotor_cfg = new TalonFXConfiguration();
    hoodMotor_cfg.Feedback.FeedbackRemoteSensorID = m_hoodCoder.getDeviceID();
    hoodMotor_cfg.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
    hoodMotor_cfg.Feedback.SensorToMechanismRatio = 1.0;
    hoodMotor_cfg.Feedback.RotorToSensorRatio = 12.8;
    m_hoodMotor.getConfigurator().apply(hoodMotor_cfg);
  }

  public void hoodBack(){
    m_hoodMotor.setPosition(0);
  }

  public void runNewShooter(double speed,double kickerSpeed) {
    System.out.println("Shooter Spin:" + speed);
    m_1shooterMotor.set(speed);
    m_2shooterMotor.set(-speed);
    m_kickerMotor.set(kickerSpeed);
  }

public void runShooterVelocity(double targetSpeed){
// Torque-current bang-bang
m_1shooterMotor.setControl(new VelocityTorqueCurrentFOC(targetSpeed));
m_2shooterMotor.setControl(new VelocityTorqueCurrentFOC(-targetSpeed));


/*    m_1shooterMotor.setControl(new VelocityTorqueCurrentFOC(0)
                  .withVelocity(targetSpeed)
                  .withFeedForward(0.1));
    m_2shooterMotor.setControl(new VelocityTorqueCurrentFOC(0)
                  .withVelocity(-targetSpeed)
                  .withFeedForward(0.1));
   */
  }
  

  public void stopNewShooter(boolean idleMode){
    System.out.println("stopping shooter");
    if(idleMode){
      m_1shooterMotor.set(ShooterConstants.IdleSpeed);
      m_2shooterMotor.set(-ShooterConstants.IdleSpeed);
    }else {
      m_1shooterMotor.stopMotor();
      m_2shooterMotor.stopMotor();
    }
    m_kickerMotor.stopMotor();
  }

  private void configureShootermotor(TalonFX motor){
    TalonFXConfigurator configurator = motor.getConfigurator();
    TalonFXConfiguration config = new TalonFXConfiguration();
    //apply bang Bang Controller
      config.Slot0.kP = 999999.0;
      config.TorqueCurrent.PeakForwardTorqueCurrent = 40.0;
      config.TorqueCurrent.PeakReverseTorqueCurrent = 0.0;
      config.MotorOutput.PeakForwardDutyCycle = 1.0;
      config.MotorOutput.PeakReverseDutyCycle = 0.0;
      
    configurator.apply(config);
    //Apply current limits
    CurrentLimitsConfigs currentLimits =  new CurrentLimitsConfigs();
    currentLimits.StatorCurrentLimit=80;
    currentLimits.SupplyCurrentLimit=60;
    configurator.apply(currentLimits);
    
  } 
  public void moveHood(double speed){
    m_hoodMotor.set(speed);
  }

  public void stopHood(){
    m_hoodMotor.stopMotor();
  }

  public Command HoodUp(){
    return Commands.startEnd(    ()->moveHood(HoodConstants.hoodUpSpeed),
                                           ()->stopHood(),
                                           this);
  }
  public Command HoodDown(){
    return Commands.startEnd(    ()->moveHood(HoodConstants.hoodDownSpeed),
                                           ()->stopHood(),
                                           this);
  }
  public Command shootCommand(){
    return Commands.startEnd(    ()->runNewShooter(ShooterConstants.shootSpeed,
                                    ShooterConstants.KickerSpeed),
                                           ()->stopNewShooter(true),
                                           this);
  }
  public Command shootonCommand(){
    return Commands.runOnce(    ()->runNewShooter(ShooterConstants.shootSpeed,
                                    ShooterConstants.KickerSpeed),this);
  }
  public Command shootoffCommand(){
    return Commands.runOnce(    ()->stopNewShooter(true),this);
  }
}