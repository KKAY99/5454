package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.TestSwerveModuleSubsystem;


public class TurnReferenceCommand extends Command {
   
  private final TestSwerveModuleSubsystem m_drive;
  private double m_position;
  
  public TurnReferenceCommand(TestSwerveModuleSubsystem subsystem,double position) {
    m_drive = subsystem;
    m_position=position;
    addRequirements(m_drive);
  }

  @Override
  public void execute() {
    
    m_drive.turntoPos(m_position);
    
   
  }
  @Override
  public void end(boolean interrupted) {
    System.out.println("stopping turn");
    m_drive.stopTurn();
  }
}
