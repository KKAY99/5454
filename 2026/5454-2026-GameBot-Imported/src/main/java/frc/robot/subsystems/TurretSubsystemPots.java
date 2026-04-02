package frc.robot.subsystems;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.*;
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
import frc.robot.Constants.TurretTrackingMethod;
import frc.robot.subsystems.shooter.TurretUtil;
import frc.robot.subsystems.shooter.TurretUtil.TargetType;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import yams.mechanisms.*;
import yams.units.EasyCRT;
import yams.units.EasyCRTConfig;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.Angle;
import static edu.wpi.first.units.Units.Rotation;
import static edu.wpi.first.units.Units.Rotations;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.configs.Slot0Configs;


public class TurretSubsystemPots extends SubsystemBase {
  private TalonFX m_turretMotor;
  private Orchestra m_robotOrch = new Orchestra();
  private TurretUtil.TargetType m_target;    
 // private CANcoder m_encoder1;
 // private CANcoder m_encoder2;
  private AnalogPotentiometer m_POTS;

    private final double kPotsLowLimit=Constants.TurretConstants.TurretLeftLimitPOTS;
  private final double kPotsHighLimit=Constants.TurretConstants.TurretRightLimitPOTS;
  private double kLowerLimit=-39;
  private double kUpperLimit=5.5;
  private final double kGearReduction=8;  //80t to 10tooth
  private final double kMotorRotationsToAngle=7.87499;//7.80;
  private final double kDegreesPerRotation=0;
  private DutyCycleOut m_TurretDutyCycleOut = new DutyCycleOut(0.0);
  private MotionMagicVoltage mmRequest = new MotionMagicVoltage(0);
 
 // private MotionMagicVelocityVoltage mmRequest = new MotionMagicVelocityVoltage (0);
  public TurretSubsystemPots(int CanId1, int potsPort) {
    SmartDashboard.putNumber("Target Turret Angle",0);
    m_target=TargetType.HUB;
    m_POTS = new AnalogPotentiometer(potsPort,1,0); 
    m_turretMotor = new TalonFX(CanId1,"5454Canivore");
    TalonFXConfiguration motorConfig = new TalonFXConfiguration();
    motorConfig.MotorOutput.NeutralMode=NeutralModeValue.Brake;
   
    m_turretMotor.getConfigurator().apply(motorConfig);
    configureMotionMagic();
   
   /* m_encoder1 = new CANcoder(encoder1ID);
    

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
  */

    }

 /*  private void setEncoderConfig(CANcoder canCoder{
    CANcoderConfiguration m_config = new CANcoderConfiguration();
    m_config.MagnetSensor.
  })*/

  private Supplier<Angle> getAngle(CANcoder encoder){
    return () -> encoder.getAbsolutePosition().getValue();
  }
  
  public void setTarget(TurretUtil.TargetType target){
    m_target=target;
  }
   public void moveTurret(double speed) 
  {
      if(atLimit(speed)){
        stopTurret();
        System.out.println("Turret Stopped / At Limit");
   
      } else {
          //hard coded for testing
           m_turretMotor.set(speed);
           //m_turretMotor.setControl(m_TurretDutyCycleOut.withOutput(speed)); 
           System.out.println("Turret Move:" + speed);
   
      }

  }
  public boolean atLimit(double speed){
    boolean returnValue=false;
    //System.out.println("Limit Check:" + speed + " -- " + m_POTS.get());
    if(speed<0 && m_POTS.get()<kPotsLowLimit){// Moving Left towards zero on POTS
        returnValue=true;
    }else if (speed>0 && m_POTS.get()>kPotsHighLimit){ //moving right towards One on POTS
        returnValue=true;
    } // Moving Left towards zero on POTS
    return returnValue;
  }
  
  public void stopTurret(){
    System.out.println("Stopping Turret");
     m_turretMotor.stopMotor();
  }
 
  public void moveMotor(double targetmotorPosition){
    if((targetmotorPosition>kLowerLimit) && (targetmotorPosition<kUpperLimit)){ 
         m_turretMotor.setControl(mmRequest.withPosition(targetmotorPosition)); 
 
    } else {
      System.out.println("Move Target out of Range");
    }
      } 
      
    
    private void trackTurretToAngle(double angle){
      SmartDashboard.putNumber("Turret Util Target Angle",angle);
      double targetPos=getTargetMotorPosition(angle);
      SmartDashboard.putNumber("Turret Util Target Pos",targetPos); 
      moveMotor(targetPos);
 
}


private void configureMotionMagic(){
  // in init function
TalonFXConfiguration talonFXConfigs = new TalonFXConfiguration();
// set slot 0 gains
Slot0Configs slot0Configs = talonFXConfigs.Slot0;
slot0Configs.kS = 0.25; // Add 0.25 V output to overcome static friction
slot0Configs.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
slot0Configs.kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
slot0Configs.kP = 2.4;//;//4.8; // A position error of 2.5 rotations results in 12 V output
slot0Configs.kI = 0.01; // no output for integrated error
slot0Configs.kD = 0.1; // A velocity error of 1 rps results in 0.1 V output

// set Motion Magic settings
MotionMagicConfigs motionMagicConfigs = talonFXConfigs.MotionMagic;
motionMagicConfigs.MotionMagicCruiseVelocity = 80;//80; 
motionMagicConfigs.MotionMagicAcceleration = 160;//50;////160;
motionMagicConfigs.MotionMagicJerk = 000; // Target jerk of 1600 rps/s/s (0.1 seconds)

m_turretMotor.getConfigurator().apply(talonFXConfigs);
} 



private double getTurretAngleFromMotor(){
  double motorPos=m_turretMotor.getPosition().getValueAsDouble();
  double returnValue=0;
  if(motorPos>=0){
     returnValue=motorPos*kDegreesPerRotation;
  } else {
     returnValue= 360-(Math.abs(motorPos)*kDegreesPerRotation);
  }
  return returnValue;

}
public double getTargetMotorPosition(double targetangle){
  double returnValue=0;
  if(targetangle>=0 && targetangle<43){
      returnValue=targetangle/kMotorRotationsToAngle;
  }else { // going negative
      returnValue=0-((360-targetangle)/kMotorRotationsToAngle); // flip the sign and go that degrees to the other direction
  }
 return returnValue;
}
public boolean isOnTargetAngle(double angle){
  return true;
}

public double getTurretPOTS(){
  return m_POTS.get();
}

public double getCurrentAngle(){
  return getTurretAngleFromMotor();
}
public double getCurrentPosition(){
  return m_turretMotor.getPosition().getValueAsDouble();
}
public void turretTrack(TargetType target){

}

/*
 Holds the turret at a specific angle using MotionMagic PID control.
 This is used during shooting to keep the turret locked in place.
 @param targetAngleDegrees The angle to hold in degrees (0-360)
 */
public void holdTurretAtAngle(double targetAngleDegrees) {
  double targetMotorPosition = getTargetMotorPosition(targetAngleDegrees);
  moveMotor(targetMotorPosition);
}
public void playMusic(String fileName){
 //       m_robotOrch.loadMusic(fileName);
 //       m_robotOrch.addInstrument(m_turretMotor);
 //      m_robotOrch.play();
    }


//imagine 100 degrees rotations is all the way to the right
//imagine 100 degrees rotations is all the way to the left

private double POTStoRotations(double POTSValue ){
  double returnValue=0;
  double kMotorRotationsToPOTSDegrees=10;  
  if ((POTSValue>0.5)){
     returnValue=POTSValue*kMotorRotationsToPOTSDegrees;
  }else {
     returnValue=-POTSValue*kMotorRotationsToPOTSDegrees;
  }
 
  return returnValue;
}
 public void homeTurret(){
  //set position of encoder to offset for POTS
  double currentPOTPosition = m_POTS.get(); 
  double calcHomePos;
  double kHomeTurretPOTS=0.851;
  double potsGap=Math.abs(kHomeTurretPOTS-currentPOTPosition);
  calcHomePos=potsGap*56.25;
  if(currentPOTPosition<kHomeTurretPOTS){
    calcHomePos=calcHomePos*-1; // go negative on position
  }
  System.out.println("Homing Position: " + calcHomePos );
  m_turretMotor.setPosition(calcHomePos,1);
  SmartDashboard.putNumber("Target Pos for 0", getTargetMotorPosition(0));
  SmartDashboard.putNumber("Target Pos for 90", getTargetMotorPosition(90));
  SmartDashboard.putNumber("Target Pos for 180",+ getTargetMotorPosition(180));
  SmartDashboard.putNumber("Target Pos for 270", getTargetMotorPosition(270));
}   

  @Override
  public void periodic(){
    SmartDashboard.putNumber("Motor Rotations",m_turretMotor.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("POTS",m_POTS.get());
    SmartDashboard.putNumber("POTS Angle",m_POTS.get()*3600/kGearReduction);
    SmartDashboard.putNumber("POTS Offset Angle",(m_POTS.get()*3600/kGearReduction)-225);
    SmartDashboard.putNumber("Motor Rotation Angle",getTurretAngleFromMotor());
    

  /*   double angle=SmartDashboard.getNumber("Target Turret Angle",0);
 
   if(!(angle==0)){
      double targetPos=getTargetMotorPosition(angle);
      System.out.println("Move Turret to "+ targetPos);
        moveMotor(targetPos);
    }  //SmartDashboard.putBoolean("AtLimit",atLimit(m_speed));
    //SmartDashboard.putNumber("POTS",m_POTS.; 
*/
  }
}