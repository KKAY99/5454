package frc.robot.commands;


import frc.robot.subsystems.PneumaticsSubsystem;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class zIntakeArmMoveCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final PneumaticsSubsystem m_subsystem;
  private final boolean m_status;
  private boolean m_executed=false;
  
   public zIntakeArmMoveCommand(PneumaticsSubsystem PneumaticsSystem,boolean status) {
    m_subsystem=PneumaticsSystem;
    m_status=status;
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
     m_subsystem.setIntakeArms(m_status);
     m_executed=true;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(final boolean interrupted) {
  
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
     return m_executed;
  }
}

