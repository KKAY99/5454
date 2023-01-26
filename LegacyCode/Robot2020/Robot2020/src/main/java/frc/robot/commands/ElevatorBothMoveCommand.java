
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


public class ElevatorBothMoveCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final double m_targetHighSpeed;
  private final double m_targetLowSpeed;
  private final ElevatorSubSystemHigh m_subsystemHigh;
  private final ElevatorSubSystemLow m_subsystemLow;
  /**
    * @param targetSpeed The speed we are setting in execute
   */
  public ElevatorBothMoveCommand(ElevatorSubSystemHigh subsystemHigh,double targetHighSpeed, ElevatorSubSystemLow subsystemLow,double targetLowSpeed) {
    m_targetHighSpeed = targetHighSpeed;
    m_targetLowSpeed=targetLowSpeed;
    m_subsystemHigh=subsystemHigh;
    m_subsystemLow=subsystemLow;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_subsystemHigh);
    addRequirements(m_subsystemLow);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println("Executing MoveBothElevator");
    m_subsystemHigh.setSpeed(m_targetHighSpeed);
    m_subsystemLow.setSpeed(m_targetLowSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(final boolean interrupted) {
    m_subsystemHigh.setSpeed(0);
    m_subsystemLow.setSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
