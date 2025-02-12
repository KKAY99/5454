package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.Constants.ElevatorConstants.ElevatorScoreLevel;
import java.util.function.Supplier;

import com.revrobotics.spark.ClosedLoopSlot;

public class ElevatorPosCommand extends Command {
  private ElevatorSubsystem m_elevator;

  private Supplier<ElevatorScoreLevel> m_scoreLevel;
  private ClosedLoopSlot m_closedLoopSlot;

  private double m_pos;

  public ElevatorPosCommand(ElevatorSubsystem elevator, double pos) {
    m_elevator=elevator;
    addRequirements(elevator);
    m_closedLoopSlot=ClosedLoopSlot.kSlot0;
    m_pos=pos;
  }

  public ElevatorPosCommand(ElevatorSubsystem elevator,Supplier<ElevatorScoreLevel> scoreLevel) {
    m_elevator=elevator;
    m_closedLoopSlot=ClosedLoopSlot.kSlot0;
    m_scoreLevel=scoreLevel;
  }

  public ElevatorPosCommand(ElevatorSubsystem elevator,Supplier<ElevatorScoreLevel> scoreLevel,ClosedLoopSlot closedLoopSlot) {
    m_elevator=elevator;
    m_scoreLevel=scoreLevel;
    m_closedLoopSlot=closedLoopSlot;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize(){
    if(m_scoreLevel!=null){
      switch(m_scoreLevel.get()){
        case L1:
        m_pos=ElevatorConstants.l1Pos;
        break;
        case L2:
        m_pos=ElevatorConstants.l2Pos;
        break;
        case L3:
        m_pos=ElevatorConstants.l3Pos;
        break;
        case L4:
        m_pos=ElevatorConstants.l4Pos;
        break;
        case RETRACT:
        m_pos=ElevatorConstants.elevatorLowLimit;
        break;
      }
    }
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
    m_elevator.set_referance(m_pos,m_closedLoopSlot);
     
    return true;
  }
}
