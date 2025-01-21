// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.EndEffectorSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class EndEffectorCommand extends Command {
  private EndEffectorSubsystem m_subSystem;
  private double m_speed;
  private int m_motor;
  /** Creates a new IntakeCommand. */
  public EndEffectorCommand(EndEffectorSubsystem system, int motor, double speed) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_subSystem = system;
    m_speed = speed;
    m_motor=motor;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_subSystem.motor_runmotor(m_motor, m_speed); 
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_subSystem.motor_stop(m_motor);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
