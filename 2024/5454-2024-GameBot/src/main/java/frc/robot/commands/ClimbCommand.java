// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimbSubsystem;

public class ClimbCommand extends Command {
  private ClimbSubsystem m_climb;

  private double m_speed;

  public ClimbCommand(ClimbSubsystem climb,double speed){
    m_climb=climb;
    m_speed=speed;
  }

  @Override
  public void initialize(){}

  @Override
  public void execute(){
    m_climb.runClimb(m_speed);
  }

  @Override
  public void end(boolean interrupted){
    m_climb.stopClimb();
  }

  @Override
  public boolean isFinished(){
    return false;
  }
}
