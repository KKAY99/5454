package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.GroundIntakeConstants;
import frc.robot.subsystems.GroundIntakeSubsystem;

public class GroundIntakeAutomatedCommand extends Command {
  private GroundIntakeSubsystem m_groundIntake;

  private double m_speed;
  private double m_endSpeed;
  private double m_position;
  private double m_endPosition;

  private enum States {
    SETPOS,WAITFORPOS,RUNINTAKE,END
  }

  private States m_currentState;

  public GroundIntakeAutomatedCommand(GroundIntakeSubsystem intake,double position,double speed,double endPos,double endSpeed) {
    m_groundIntake = intake;
    m_speed = speed;
    m_position = position;
    m_endSpeed=endSpeed;
    m_endPosition=endPos;

    addRequirements(m_groundIntake);
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize(){
    m_currentState=States.SETPOS;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_groundIntake.runIntake(m_endSpeed);
    m_groundIntake.resetPID();
    m_groundIntake.togglePID(m_endPosition);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    boolean returnValue=false;

    switch(m_currentState){
      case SETPOS:
        m_groundIntake.resetPID();
        m_groundIntake.togglePID(m_position);

        m_currentState=States.WAITFORPOS;
      break;
      case WAITFORPOS:
        if(m_groundIntake.getAbsoluteEncoderPos()>m_position-GroundIntakeConstants.posDeadband&&
          m_groundIntake.getAbsoluteEncoderPos()<m_position+GroundIntakeConstants.posDeadband){
            m_currentState=States.RUNINTAKE;
        }
      break;
      case RUNINTAKE:
        m_groundIntake.runIntake(m_speed);
      break;
      case END:
        returnValue=true;
    }

    return returnValue;
  }
}
