package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DunkinDonutSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.Constants.DunkinDonutConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.ElevatorConstants.ElevatorScoreLevel;
import frc.robot.subsystems.ElevatorSubsystem;

public class DunkinDonutPosCommand extends Command {
  private DunkinDonutSubsystem m_dunkin;
  private ElevatorSubsystem m_elevator;

  private Supplier<ElevatorScoreLevel> m_scoreLevel;

  private double m_pos;
  private double m_startTime;

  public DunkinDonutPosCommand(DunkinDonutSubsystem dunkin, double pos) {
    m_dunkin=dunkin;
    m_pos=pos;
  }

  public DunkinDonutPosCommand(DunkinDonutSubsystem dunkin,ElevatorSubsystem elevator,Supplier<ElevatorScoreLevel> scoreLevel) {
    m_dunkin=dunkin;
    m_elevator=elevator;
    m_scoreLevel=scoreLevel;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize(){
    m_startTime=Timer.getFPGATimestamp();

    if(m_scoreLevel!=null){
      switch(m_scoreLevel.get()){
        case L1:
        m_pos=DunkinDonutConstants.l1Pos;
        break;
        case L2:
        m_pos=DunkinDonutConstants.l2Pos;
        break;
        case L3:
        m_pos=DunkinDonutConstants.l3Pos;
        break;
        case L4:
        m_pos=DunkinDonutConstants.l4Pos;
        break;
        case RETRACT:
        m_pos=0;
        break;
      }
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished(){
    boolean returnValue=false;

    if(m_elevator.getRelativePos()<ElevatorConstants.aboveTroughPos){
      m_dunkin.set_referance(m_pos);
      returnValue=true;
    }
    
    return returnValue;
  }
}
