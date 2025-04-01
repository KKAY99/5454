package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.GroundIntakeSubsystem;

public class GroundIntakeAutomatedCommand extends Command {
  private GroundIntakeSubsystem m_groundIntake;
  private double m_speed;
  private double m_position;

  public GroundIntakeAutomatedCommand(GroundIntakeSubsystem intake, double position, double speed) {
    m_groundIntake = intake;
    m_speed = speed;
    m_position = position;
    addRequirements(m_groundIntake);
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize(){
    m_groundIntake.movetoPosition(m_position); 
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  
    m_groundIntake.runIntake(m_speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_groundIntake.stopIntake();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
