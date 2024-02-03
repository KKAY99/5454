package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ConveyorSubsystem;


public class ConveyCommand extends Command{
    // Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

  private ConveyorSubsystem m_convey;

  private double m_speed;

  public ConveyCommand(ConveyorSubsystem convey,double speed){
    m_convey=convey;
    m_speed=speed;
  }

  @Override
  public void initialize(){}

  @Override
  public void execute(){
    m_convey.runConveyor(m_speed);
  }

  @Override
  public void end(boolean interrupted){
    m_convey.stopConveyer();
  }

  @Override
  public boolean isFinished(){
    return false;
  }
}

