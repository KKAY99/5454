package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.TestSwerveModuleSubsystem;


public class TurnCommand extends Command {
   
  private final TestSwerveModuleSubsystem m_drive;
  private double m_speed; 
  public TurnCommand(TestSwerveModuleSubsystem subsystem,double speed) {
    m_drive = subsystem;
    m_speed=speed;
    addRequirements(m_drive);
  }

  @Override
  public void execute() {
    m_drive.turn(m_speed);   
    
   
  }
  @Override
  public void end(boolean interrupted) {
  m_drive.stopTurn();
  }
}
