package frc.robot.commands;

import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.GroundIntakeSubsystem;

public class GroundIntakeRotateCommand extends Command {
  private GroundIntakeSubsystem m_groundIntake;
  private Double m_speed;

  public GroundIntakeRotateCommand(GroundIntakeSubsystem intake, Double speed) {
    m_groundIntake = intake;
    m_speed = speed;
    addRequirements(m_groundIntake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize(){
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_groundIntake.runRotate(m_speed);;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_groundIntake.stopRotate();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
