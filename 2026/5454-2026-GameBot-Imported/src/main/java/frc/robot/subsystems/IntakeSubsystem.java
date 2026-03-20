// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utilities.ObsidianCANSparkMax;
import edu.wpi.first.wpilibj.DigitalInput;

import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class IntakeSubsystem extends SubsystemBase {
  private TalonFX m_intakeMotor;
  private ObsidianCANSparkMax m_fold;
  private DigitalInput m_intakeSwitch;
  private boolean m_IntakeOutMode = false; //starts in in Mode and gets reversed on first call
  private boolean m_homed=false;
  //private SparkAbsoluteEncoder m_encoder;

  public IntakeSubsystem(int CanId1, int CanId2) {
    m_intakeMotor = new TalonFX(CanId1);
    m_intakeMotor.setNeutralMode(NeutralModeValue.Brake
    );
    m_fold = new ObsidianCANSparkMax(CanId2, MotorType.kBrushless, true,80);
    m_intakeSwitch = new DigitalInput(IntakeConstants.intakeSwitchDIO);
  }

  public void outFold(double speed) {
    m_fold.set(speed);
  }

  public void inFold(double speed) {
    m_fold.set(-speed);
  }

  public boolean isIntakeSwitched() {
    return m_intakeSwitch.get();
  }

  public void homeIntake(double maxHomeTime){
    //pull intake until we hit limit switch and then reset position
     double startTime = Timer.getFPGATimestamp();
     double endTime = startTime + maxHomeTime;
    System.out.println("Intake is starting homing");
    while(!isIntakeSwitched() && Timer.getFPGATimestamp()<endTime){
     m_fold.set(-Constants.IntakeConstants.foldHomeSpeed);
    }
    m_fold.stopMotor(); // stop intake
    m_fold.getEncoder().setPosition(0.0); // reset encoder to zero at home in
    System.out.println("Intake has homed");
    SetIntakeInMode();
    m_homed=true;
  }
  public void SetIntakeInMode(){
      m_IntakeOutMode=false;
  }
  
  public void SetIntakeOutMode(){
      m_IntakeOutMode=true;
  }
  public void toggleIntakeMode() {
    m_IntakeOutMode=!m_IntakeOutMode;

  }

  public void stopFold() {
    m_fold.stopMotor();
  }

  public void runIntake(double speed) {
    m_intakeMotor.set(speed);
  }

  public void stopIntake(){
    System.out.println("Stopping Intake");
    m_intakeMotor.stopMotor();
  }

  public Command foldCommand(double speed){
      return Commands.startEnd(    ()->runFoldManual(speed),
                                          ()->stopFold(),
                                          this);
  }
 
  public Command outtakeCommand(){
      return Commands.startEnd(    ()->runIntake(IntakeConstants.outtakeSpeed),
                                          ()->stopIntake(),
                                          this);
  }
  
  public Command intakeCommand(){
      return Commands.startEnd(    ()->runIntake(IntakeConstants.highSpeed),
                                          ()->stopIntake(),
                                          this);
  }

  public Command intakeonCommand(){
    return Commands.runOnce(    ()->runIntake(IntakeConstants.highSpeed),this);
  }

  public Command intakeoffCommand(){
    return Commands.runOnce(    ()->stopIntake(),this);
  }

  public boolean intakeCurrentLimitCheck(double ampCheck){
    return m_fold.getOutputCurrent()>ampCheck;
  }

  private double getFoldState() {
    return m_fold.getOutputCurrent();
  }

  public boolean hasHomed(){
    return m_homed;
  }
  public boolean isIntakeOutMode(){
    return m_IntakeOutMode;
  }
  public boolean isinNoFlyZone(){
    boolean returnValue=false;
    //NO FLY ZONE IS DISABLED DUE TO MECH FIXES
    double currentPos=Math.abs(m_fold.getEncoder().getPosition());
    if(currentPos<Constants.IntakeConstants.intakeRollStop){
      returnValue=true;
    }
    return returnValue;
  }
  private void runFoldManual(double Speed){
    m_fold.set(Speed);
    if((Speed<0) && !isinNoFlyZone()){
      m_intakeMotor.set(IntakeConstants.highSpeed);   
    } else if((Speed>0) && isinNoFlyZone()){
      m_intakeMotor.stopMotor();   
    }
  }

  public boolean isAtInLimit(){
    boolean returnValue=false;
    if(Math.abs(m_fold.getEncoder().getPosition())<Constants.IntakeConstants.intakeinPos){
        returnValue=true;
    }
    return returnValue;
  }
   
  public boolean isAtOutLimit(){
    boolean returnValue=false;
    if(Math.abs(m_fold.getEncoder().getPosition())>Constants.IntakeConstants.intakeEndStop){
        returnValue=true;
    }
    return returnValue;
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Intake Fold Current", m_fold.getOutputCurrent());
    SmartDashboard.putNumber("Intake Position", m_fold.getEncoder().getPosition());
    SmartDashboard.putBoolean("Intake Switch",m_intakeSwitch.get());
    // System.out.println("Amp " + m_fold.getOutputCurrent());
    // This method will be called once per scheduler run
  }
}