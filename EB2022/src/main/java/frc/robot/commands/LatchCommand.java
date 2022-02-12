package frc.robot.commands;


import frc.robot.subsystems.PneumaticsSubsystem;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class LatchCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final PneumaticsSubsystem m_subsystem;
  
  
  /**
    * @param targetSpeed The speed we are setting in execute
   */
  public LatchCommand(PneumaticsSubsystem PneumaticsSystem) {
    m_subsystem=PneumaticsSystem;
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
    if (m_subsystem.getLatchStatus()){
      m_subsystem.setLatch(false);
    } else {
      m_subsystem.setLatch(true);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(final boolean interrupted) {
  
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

