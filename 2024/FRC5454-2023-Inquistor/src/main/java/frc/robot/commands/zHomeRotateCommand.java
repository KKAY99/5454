package frc.robot.commands;
//TODO MIGRATE HOMING
    
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.Rotate;
import frc.robot.subsystems.RotateArmSubsystem;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;

/** An example command that uses an example subsystem. */
 public class zHomeRotateCommand extends CommandBase {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  //private final PaddleSubsystem m_IntakeSubsystem;
  private final double m_speed;
  private final double m_position;
  private final double m_homeTimeOut;
  private double m_StartTime=0;
  private final RotateArmSubsystem m_Rotate;
  private PIDController m_PidController;
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  
  public zHomeRotateCommand(RotateArmSubsystem rotate,double position, double speed, double homeTimeOut) { 
    m_speed = speed;
    m_position=position;
    m_Rotate = rotate;
    m_homeTimeOut=homeTimeOut;
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_StartTime = Timer.getFPGATimestamp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_Rotate.stopRotate();
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    boolean returnValue=false;
    double currentTime=Timer.getFPGATimestamp();
   // System.out.print("Looping " + currentTime + " "+ m_StartTime + " " + m_homeTimeOut);
    if(currentTime>m_StartTime+m_homeTimeOut){
        returnValue=true;  //Time Out period has happened
    }
    if(m_Rotate.hitHomeAngle()) {
        returnValue=true;  // end because hit limit switch
    }else { //move to position
      if(m_Rotate.getAbsolutePos()<m_position){
        m_Rotate.rotate(m_speed);
      }else{
        m_Rotate.rotate(0-m_speed);
      }
      
    }
    if(returnValue){
      m_Rotate.stopRotate();
      m_Rotate.SetZero();
      m_Rotate.setHomed(true);
    }
    return returnValue;
  }
 }
