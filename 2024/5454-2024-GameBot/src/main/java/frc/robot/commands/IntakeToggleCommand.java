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

  private double m_speed;

  public IntakeToggleCommand(IntakeSubsystem intake,double speed){
    m_intake=intake;
    m_speed=speed;

    addRequirements(m_intake);
  }

  @Override
  public void initialize(){}

  @Override
  public void execute(){
    m_intake.ToggleIntake(m_speed);
  }

  @Override 
  public boolean isFinished() {
      // TODO Auto-generated method stub
      return true;
  }
}
