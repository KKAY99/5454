package frc.robot.commands;
    
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ElevatorSubsystem;
import edu.wpi.first.wpilibj.Timer;

/** An example command that uses an example subsystem. */
 public class zHomeElevatorCommand extends CommandBase {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  //private final PaddleSubsystem m_IntakeSubsystem;
  private final double m_speed;
  private final double m_homeTimeOut;
  private double m_StartTime=0;
  private final ElevatorSubsystem m_Elevator;
 
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  
  public zHomeElevatorCommand(ElevatorSubsystem elevator,double speed, double homeTimeOut) { 
    m_speed = speed;
    m_Elevator = elevator;
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
    m_Elevator.runWithOutLimit(m_speed);
    //  m_Elevator.runWithOutLimit(m_speed); // move arms at home speed until isfinished
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_Elevator.stop();
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
    if(m_Elevator.hasHitPhysicalLimitSwitch()) {
        returnValue=true;  // end because hit limit switch
    }
    if(returnValue){
      m_Elevator.stop();
      m_Elevator.setZero();
      m_Elevator.setHomed(true);
    }
    return returnValue;
  }
 }
