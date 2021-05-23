
/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.robot.subsystems.ElevatorSubSystemHigh;

import frc.robot.subsystems.ElevatorSubSystemLow;

import edu.wpi.first.wpilibj2.command.CommandBase;


public class ElevatorMoveBoth extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ElevatorSubSystemHigh m_high_subsystem = new ElevatorSubSystemHigh();
  private final ElevatorSubSystemLow m_low_subsystem = new ElevatorSubSystemLow();
  private final double m_targetSpeed;
  /**
    * @param targetSpeed The speed we are setting in execute
   */
  public ElevatorMoveBoth(final double targetSpeed) {
    m_targetSpeed = targetSpeed;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_high_subsystem);
    addRequirements(m_low_subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println("Executing MoveElevator");
    m_high_subsystem.setSpeed(m_targetSpeed);
    m_low_subsystem.setSpeed(m_targetSpeed);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(final boolean interrupted) {
    m_low_subsystem.setSpeed(0);
    m_high_subsystem.setSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
