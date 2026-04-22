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
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import static edu.wpi.first.units.Units.Rotations;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import frc.robot.Constants.HoodConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.utilities.ObsidianCANSparkMax;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTableInstance.NetworkMode;
import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class NewShooterSubsystemOld extends SubsystemBase {
    private TalonFX m_1shooterMotor;
    private TalonFX m_2shooterMotor;
    private TalonFX m_hoodMotor;
    private CANcoder m_hoodCoder;
    private Pose2d m_pose;
//    private PositionVoltage m_request= new PositionVoltage(0).withSlot(0);
   private MotionMagicDutyCycle mmRequest = new MotionMagicDutyCycle(0);

    //private TalonFX m_kickerMotor;
    private ObsidianCANSparkMax m_kickerMotor;
  public NewShooterSubsystemOld(int shooter1CANID, int shooter2CANID, int kickerCANID,int hoodCANID) {
    m_hoodCoder = new CANcoder(Constants.HoodConstants.hoodCoderCANID);
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
    hoodMotor_cfg.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder; // Set the feedback source to the CANcoder
    m_hoodMotor.getConfigurator().apply(hoodMotor_cfg); // Apply the configuration to the hood motor
    
    m_hoodMotor.setNeutralMode(NeutralModeValue.Brake);
   
    configureMotionMagic(); //on Hood Motor //
    //configureHoodMotor();
    
  }
private void configureHoodMotor(){
     TalonFXConfiguration config = new TalonFXConfiguration();

        config.Slot0.kP = 0.1;
        config.Slot0.kI = 0.0;
        config.Slot0.kD = 1;
        config.Slot0.kS = 0.25; // Friction feedforward
        config.Slot0.kV = 0.12;
        config.Slot0.kA = 0.0;

        // 4. Set Neutral Mode (Brake is usually best for hoods)
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        m_hoodMotor.getConfigurator().apply(config);
}
private void configureMotionMagic(){
  // in init function
TalonFXConfiguration talonFXConfigs = new TalonFXConfiguration();
// set slot 0 gains
Slot0Configs slot0Configs = talonFXConfigs.Slot0;
slot0Configs.kS = 0.25; // Add 0.25 V output to overcome static friction
slot0Configs.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
slot0Configs.kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
slot0Configs.kP = 1.0;//4.8; 
slot0Configs.kI = 0; // no output for integrated error
slot0Configs.kD = 0.1; // A velocity error of 1 rps results in 0.1 V output

// set Motion Magic settings
MotionMagicConfigs motionMagicConfigs = talonFXConfigs.MotionMagic;
motionMagicConfigs.MotionMagicCruiseVelocity = 4;//80; // Target cruise velocity of 80 rps
motionMagicConfigs.MotionMagicAcceleration = 8;////160; // Target acceleration of 160 rps/s (0.5 seconds)
motionMagicConfigs.MotionMagicJerk = 16; // Target jerk of 1600 rps/s/s (0.1 seconds)

m_hoodMotor.getConfigurator().apply(talonFXConfigs);
}

  public double getHoodPos(){
    double currentPos=m_hoodCoder.getAbsolutePosition().getValueAsDouble();
    if(currentPos<0.95){
      return currentPos;
    }else {
      //went slightly below zero
      return 0.0;
    }
  }
  public void setPose(Pose2d pose){
      m_pose=pose;
   //   //System.out.println("Pose Set" + m_pose.getX());
      
    }
    public Pose2d getRobotPose(){
   //   //System.out.println("Reuurning Pose : " + m_pose.getX()); 
      return m_pose;
    }
  
  
  public boolean checkHoodPos(double hoodTarget, double hoodSpeed,double deadband){
    double hoodDiff=Math.abs(getHoodPos()-hoodTarget);
    return (hoodDiff<=deadband);
  }


  public void poormanHoldHoodPos(double hoodTarget, double hoodSpeed,double deadband){
    double currentPos = getHoodPos();
    double hoodDiff=Math.abs(currentPos-hoodTarget);    
        if(hoodDiff>deadband){
            if(currentPos>hoodTarget){
                m_hoodMotor.set(-hoodSpeed);
            }else {
                m_hoodMotor.set(hoodSpeed);
            }            
          }else{
            m_hoodMotor.stopMotor();
          }
  }

  public void holdHoodPosMotionMagic(double hoodTarget) {
    m_hoodMotor.setControl(mmRequest.withPosition(hoodTarget));
  }

  public void primeMotors(double primeSpeed){
    //System.out.println("Shooter Priming:" + primeSpeed);
    runShooterVelocity(primeSpeed);
  }
  public void hoodMoveToPosition(double hoodTarget, double hoodSpeed){
    double startTime=Timer.getFPGATimestamp();
    double kTimeLimit =1;
    
    double deadband = Constants.HoodConstants.hoodDeadband;
    while (!checkHoodPos(hoodTarget,hoodSpeed,deadband)  && 
    Timer.getFPGATimestamp()<(startTime+kTimeLimit)){
      poormanHoldHoodPos(hoodTarget,hoodSpeed,deadband);
    }
  }
  
  public void hoodMoveToZero(){
    double startTime=Timer.getFPGATimestamp();
    double kTimeLimit =1;
    double hoodTarget=0;
    double hoodSpeed=Constants.HoodConstants.hoodSpeed;
    double deadband = Constants.HoodConstants.hoodDeadband;
    while (!checkHoodPos(hoodTarget,hoodSpeed,deadband)
      && Timer.getFPGATimestamp()<(startTime+kTimeLimit))
        {      poormanHoldHoodPos(hoodTarget,hoodSpeed,deadband);
    }
    stopHood();
  }
 

  public void hoodMove(double hoodSpeed){
    m_hoodMotor.set(hoodSpeed);
  }

  public void runNewShooter(double speed,double kickerSpeed) {
    //System.out.println("Shooter Spin:" + speed);
    runShooterVelocity(speed);
    m_kickerMotor.set(kickerSpeed);
  }
public void runKicker(double kickerSpeed){
  m_kickerMotor.set(kickerSpeed);
}
public boolean atTargetSpeed(double targetSpeed){
  double currentSpeed=m_1shooterMotor.getVelocity().getValueAsDouble();
  double speedDiff = Math.abs(currentSpeed-targetSpeed);
  //System.out.println("Compare Shooter Speed:" + currentSpeed + " - " + targetSpeed);
  if(currentSpeed>targetSpeed){
    //allow greater deadband for going over shot
     return (speedDiff<Constants.ShooterConstants.shooterVelocityHighDeadband);
  } else {
       return (speedDiff<Constants.ShooterConstants.shooterVelocityLowDeadband);
  }
 
}
public void runShooterVelocity(double targetSpeed){
// Torque-current bang-bang
//m_1shooterMotor.setControl(new MotionMagicVelocityVoltage(targetSpeed));
//m_1shooterMotor.setControl(new MotionMagicVelocityVoltage(-targetSpeed));
//replaced FOC 
m_1shooterMotor.setControl(new VelocityVoltage(targetSpeed));
m_2shooterMotor.setControl(new VelocityVoltage(-targetSpeed));


/*    m_1shooterMotor.setControl(new VelocityTorqueCurrentFOC(0)
                  .withVelocity(targetSpeed)
                  .withFeedForward(0.1));
    m_2shooterMotor.setControl(new VelocityTorqueCurrentFOC(0)
                  .withVelocity(-targetSpeed)
                  .withFeedForward(0.1));
   */
  }
  

  public void stopNewShooter(boolean idleMode){
    //System.out.println("stopping shooter");
    //demo mode change 
    idleMode=false;
    
    if(idleMode){
    
      runShooterVelocity(Constants.ShooterConstants.IdleSpeed);
    }else {
      m_1shooterMotor.stopMotor();
      m_2shooterMotor.stopMotor();
    }
    m_kickerMotor.stopMotor();
    hoodMoveToZero();

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
      config.TorqueCurrent.PeakForwardTorqueCurrent = 800.0;
      config.TorqueCurrent.PeakReverseTorqueCurrent = -800.0;
      config.MotorOutput.PeakForwardDutyCycle = 1.0;
      config.MotorOutput.PeakReverseDutyCycle = -1.0;
      
    configurator.apply(config);
    //Apply current limits
    CurrentLimitsConfigs currentLimits =  new CurrentLimitsConfigs();
    currentLimits.StatorCurrentLimit=80;
    currentLimits.SupplyCurrentLimit=60;
    configurator.apply(currentLimits);
  }
  
  public void moveHood(double speed){
   // m_hoodMotor.set(speed);
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
  public Command hoodHome(){
    return Commands.runOnce(    ()->hoodMoveToZero(),
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
public Command shutdownCommand(){
    return Commands.runOnce(    ()->stopNewShooter(false),this);
  }
    @Override
  public void periodic(){
    SmartDashboard.putNumber("Shooter Velocity", m_1shooterMotor.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber("Hood CanCoder Value",m_hoodCoder.getAbsolutePosition().getValueAsDouble());  
    SmartDashboard.putNumber("Hood 'Pos'",getHoodPos());  
    Logger.recordOutput("Shooter/ShooterMotor1Velocity", m_1shooterMotor.getVelocity().getValueAsDouble());
    Logger.recordOutput("Shooter/ShooterMotor2Velocity",  m_2shooterMotor.getVelocity().getValueAsDouble());
    Logger.recordOutput("Shooter/ShooterMotor1Current", m_1shooterMotor.getSupplyCurrent().getValueAsDouble());
    Logger.recordOutput("Shooter/ShooterMotor2Current",  m_2shooterMotor.getSupplyCurrent().getValueAsDouble());
    Logger.recordOutput("Shooter/Hood CanCoder Value", m_hoodCoder.getAbsolutePosition().getValueAsDouble());
    Logger.recordOutput("Shooter/Hood 'Pos'",getHoodPos());
   
  }
}