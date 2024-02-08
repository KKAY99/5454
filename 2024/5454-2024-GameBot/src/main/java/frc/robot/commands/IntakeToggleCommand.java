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

  public IntakeToggleCommand(IntakeSubsystem intake){
    m_intake=intake;
  }

  @Override
  public void initialize(){}

  @Override
  public void execute(){
    m_intake.ToggleIntake(Constants.IntakeConstants.intakeSpeed);
  }

  @Override 
  public boolean isFinished() {
      // TODO Auto-generated method stub
      return true;
  }
}
