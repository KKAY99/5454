// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
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
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
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
 
    private MotionMagicVoltage mmRequest = new MotionMagicVoltage(0);
 
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
    m_hoodMotor.setNeutralMode(NeutralModeValue.Brake);
    configureMotionMagic(); //on Hood Motor
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
    hoodMotor_cfg.Feedback.SensorToMechanismRatio = 1.333;
    hoodMotor_cfg.Feedback.RotorToSensorRatio = 12.8;
    m_hoodMotor.getConfigurator().apply(hoodMotor_cfg);
  }

private void configureMotionMagic(){
  // in init function
TalonFXConfiguration talonFXConfigs = new TalonFXConfiguration();
// set slot 0 gains
Slot0Configs slot0Configs = talonFXConfigs.Slot0;
slot0Configs.kS = 0.25; // Add 0.25 V output to overcome static friction
slot0Configs.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
slot0Configs.kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
slot0Configs.kP = 4.8; 
slot0Configs.kI = 0; // no output for integrated error
slot0Configs.kD = 0.1; // A velocity error of 1 rps results in 0.1 V output

// set Motion Magic settings
MotionMagicConfigs motionMagicConfigs = talonFXConfigs.MotionMagic;
motionMagicConfigs.MotionMagicCruiseVelocity = 40;//80; // Target cruise velocity of 80 rps
motionMagicConfigs.MotionMagicAcceleration = 80;////160; // Target acceleration of 160 rps/s (0.5 seconds)
motionMagicConfigs.MotionMagicJerk = 1600; // Target jerk of 1600 rps/s/s (0.1 seconds)

m_hoodMotor.getConfigurator().apply(talonFXConfigs);
}

  public double getHoodPos(){
     return m_hoodMotor.getPosition().getValueAsDouble();
  }
  public void hoodBack(){
//    m_hoodMotor.setPosition(0);
  }
  public void hoodMovetoPosition(double hoodTarget){
    //m_hoodMotor.setPosition(hoodTarget);
    System.out.println("Move Hood to " + hoodTarget);
    m_hoodMotor.setControl(mmRequest.withPosition(hoodTarget)); 
 
  }
  public void hoodMove(double hoodSpeed){
    m_hoodMotor.set(hoodSpeed);
  }

  public void runNewShooter(double speed,double kickerSpeed) {
    System.out.println("Shooter Spin:" + speed);
    runShooterVelocity(speed);
    m_kickerMotor.set(kickerSpeed);
  }

public boolean atTargetSpeed(double targetSpeed){
  double currentSpeed=m_1shooterMotor.getVelocity().getValueAsDouble();
  double speedDiff = Math.abs(currentSpeed-targetSpeed);

  return (speedDiff<Constants.ShooterConstants.shooterVelocityDeadband);
}
public void runShooterVelocity(double targetSpeed){
// Torque-current bang-bang
//m_1shooterMotor.setControl(new MotionMagicVelocityVoltage(targetSpeed));
//m_1shooterMotor.setControl(new MotionMagicVelocityVoltage(-targetSpeed));
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
      runShooterVelocity(Constants.ShooterConstants.IdleSpeed);
    }else {
      m_1shooterMotor.stopMotor();
      m_2shooterMotor.stopMotor();
    }
    m_kickerMotor.stopMotor();
  }

  
  private void PIDconfigureShootermotor(TalonFX motor){
    TalonFXConfigurator configurator = motor.getConfigurator();
    TalonFXConfiguration config = new TalonFXConfiguration();
    //apply bang Bang Controller
      config.Slot0.kP = 0.1;
      config.Slot0.kV = 0.1;//0.12; // Velocity feedforward
  
      
    configurator.apply(config);
    //Apply current limits
    CurrentLimitsConfigs currentLimits =  new CurrentLimitsConfigs();
    currentLimits.StatorCurrentLimit=80;
    currentLimits.SupplyCurrentLimit=60;
    configurator.apply(currentLimits);
    
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
    @Override
  public void periodic(){
    SmartDashboard.putNumber("Shooter Velocity", m_1shooterMotor.getVelocity().getValueAsDouble());
  }
}