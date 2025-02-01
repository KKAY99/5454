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
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_elevator.set_referance(m_pos);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
     m_elevator.reset_referance();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    boolean returnValue=false;

    if (m_elevator.get_motor1pos()>m_pos-ElevatorConstants.posDeadband&&m_elevator.get_motor1pos()<m_pos+ElevatorConstants.posDeadband){
       returnValue=true;
    }
     
    return returnValue;
  }
}
