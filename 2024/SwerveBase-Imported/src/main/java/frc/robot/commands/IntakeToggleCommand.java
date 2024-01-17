// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.Constants;

public class IntakeToggleCommand extends Command{
  private IntakeSubsystem m_intake;

  private boolean m_toggle;

  public IntakeToggleCommand(IntakeSubsystem intake,boolean toggle){
    m_intake=intake;
    m_toggle=toggle;

  }

  @Override
  public void initialize(){}

  @Override
  public void execute(){
    if(m_toggle){
      m_intake.run(Constants.IntakeConstants.intakeSpeed);  
    }else{
      m_intake.stop();
    }
  }
}
