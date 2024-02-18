// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeToggleCommand extends Command{
  private IntakeSubsystem m_intake;

  
  private double m_speed;
  private boolean m_isRunning;
  public IntakeToggleCommand(IntakeSubsystem intake,double speed){
    m_intake=intake;
    m_speed=speed;
    m_isRunning=false;
    addRequirements(m_intake);
  }

  @Override
  public void initialize(){}

  @Override
  public void execute(){
    m_isRunning=true;
    m_intake.ToggleIntake(m_speed);
  }
  
  @Override public void end(boolean interrupted){
   m_isRunning=false; 
     Logger.recordOutput("Intake/IntakeToggleCommand",m_isRunning);

  }

  @Override 
  public boolean isFinished() {
      // TODO Auto-generated method stub
      Logger.recordOutput("Intake/IntakeToggleCommand",m_isRunning);

      return true;
  }
}
