// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DunkinDonutSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DunkinDonutAlgeaCommand extends Command {
  private DunkinDonutSubsystem m_dunkin;
  private double m_speed;
  private boolean m_toggle=false;
 
  public DunkinDonutAlgeaCommand(DunkinDonutSubsystem dunkin,double speed,boolean toggle) {
    m_dunkin = dunkin;
    m_speed = speed;
    m_toggle=toggle;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(m_toggle){
      m_dunkin.algeaToggle(m_speed);
    }else{
      m_dunkin.runAlgaeMotor(m_speed);
    }
  }
   
  @Override
  public void end(boolean interrupted) {
    if(!m_toggle){
      m_dunkin.stopAlgeaMotor();
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(m_toggle){
      return true;
    }else{
      return false;
    }
  }
}
