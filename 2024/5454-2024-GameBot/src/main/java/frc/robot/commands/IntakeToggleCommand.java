// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeToggleCommand extends Command{
  private IntakeSubsystem m_intake;
  private XboxController m_xboxController;
  
  private double m_speed;
  private boolean m_isRunning;
  private boolean m_shouldToggle;

  public IntakeToggleCommand(IntakeSubsystem intake,double speed,boolean shouldToggle){
    m_intake=intake;
    m_shouldToggle=shouldToggle;
    m_speed=speed;
    m_isRunning=false;
    addRequirements(m_intake);
  }

  public IntakeToggleCommand(IntakeSubsystem intake,double speed,XboxController xboxController,boolean shouldToggle){
    m_intake=intake;
    m_shouldToggle=shouldToggle;
    m_speed=speed;
    m_xboxController=xboxController;
    m_isRunning=false;
    addRequirements(m_intake);
  }

  @Override
  public void initialize(){}

  @Override
  public void execute(){
    m_isRunning=true;
    if(m_shouldToggle){
      m_intake.ToggleIntake(m_speed,m_xboxController);
    }
  }
  
  @Override public void end(boolean interrupted){
     m_isRunning=false; 
     if(!m_shouldToggle){
      m_intake.ResetToggleBoolean();
     }
     Logger.recordOutput("Intake/IntakeToggleCommand",m_isRunning);
     if(!m_shouldToggle){
      m_intake.stopIntake();
     }

  }

  @Override 
  public boolean isFinished() {
    boolean returnValue=m_shouldToggle;
      Logger.recordOutput("Intake/IntakeToggleCommand",m_isRunning);
      if(!m_shouldToggle){
        m_intake.runIntake(m_speed);
      }
      
      return returnValue;
  }
}
