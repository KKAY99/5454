
/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.robot.subsystems.ElevatorSubSystemLow;

import edu.wpi.first.wpilibj2.command.CommandBase;


public class ElevatorLowMoveCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final double m_targetSpeed;
  private final ElevatorSubSystemLow m_subsystem;
  /**
    * @param targetSpeed The speed we are setting in execute
   */
  public ElevatorLowMoveCommand(ElevatorSubSystemLow subsystem,final double targetSpeed) {
    m_subsystem=subsystem;
    m_targetSpeed = targetSpeed;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    m_subsystem.setSpeed(m_targetSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(final boolean interrupted) {
    m_subsystem.setSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
