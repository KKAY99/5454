// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class IntakeCommand extends CommandBase {

  private IntakeSubsystem m_subsystem;
  private double m_speed;

  /** Creates a new testCommand. */
  public IntakeCommand(IntakeSubsystem subsystem,double speed) {
    // Use addRequirements() here to declare subsystem dependencies.
    
    m_subsystem=subsystem;
    m_speed=speed;
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println("Execute Command "+ m_speed);
    m_subsystem.run(m_speed);
     
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_subsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
   
  }
}