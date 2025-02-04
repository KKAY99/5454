package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.ElevatorSubsystem;

public class ElevatorPosCommand extends Command {
  private ElevatorSubsystem m_elevator;
  private double m_pos;

  public ElevatorPosCommand(ElevatorSubsystem elevator, double pos) {
    m_elevator = elevator;
    m_pos = pos;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute(){
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    m_elevator.set_referance(m_pos);
     
    return true;
  }
}
