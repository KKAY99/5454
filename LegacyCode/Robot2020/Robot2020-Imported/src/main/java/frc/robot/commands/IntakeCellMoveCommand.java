
/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.robot.subsystems.IntakeSubSystem;
import edu.wpi.first.wpilibj2.command.CommandBase;


public class IntakeCellMoveCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final IntakeSubSystem m_subsystem;
  private final double m_targetSpeed;
  /**
   * @param subsystem The subsystem used by this command.
   * @param targetSpeed The speed we are setting in execute
   */
  public IntakeCellMoveCommand(IntakeSubSystem subsystem,double targetSpeed) {
    m_subsystem = subsystem;
    m_targetSpeed=targetSpeed;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("Init IntakeMove");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
     System.out.println("Executing Move Intake");
      m_subsystem.setSpeed(m_targetSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_subsystem.setSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
