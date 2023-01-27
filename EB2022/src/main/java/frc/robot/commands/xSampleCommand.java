package frc.robot.commands;


import frc.robot.subsystems.xSampleSubsystem;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class xSampleCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final xSampleSubsystem m_subsystem;
  private final double m_targetSpeed;
  
  /**
    * @param targetSpeed The speed we are setting in execute
   */
  public xSampleCommand(xSampleSubsystem xSamplesubsystem,final double targetSpeed) {
    m_subsystem=xSamplesubsystem;
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

