package frc.robot.commands;

import java.util.function.Supplier;

import com.revrobotics.spark.ClosedLoopSlot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.ElevatorConstants.ElevatorScoreLevel;
import frc.robot.Constants.NotificationLevel;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.utilities.Elastic;

public class ElevatorPosCommand extends Command {


  private ElevatorSubsystem m_elevator;

  private Supplier<ElevatorScoreLevel> m_scoreLevel;
  private ClosedLoopSlot m_closedLoopSlot;

  private double m_pos;

  private String elevatorlevel;

  Elastic.Notification notification = new Elastic.Notification(NotificationLevel.INFO, "Elevator level", elevatorlevel);

  public ElevatorPosCommand(ElevatorSubsystem elevator, double pos) {
    m_elevator=elevator;
    m_closedLoopSlot=ClosedLoopSlot.kSlot0;
    m_pos=pos;

    addRequirements(elevator);
  }

  public ElevatorPosCommand(ElevatorSubsystem elevator,Supplier<ElevatorScoreLevel> scoreLevel) {
    m_elevator=elevator;
    m_closedLoopSlot=ClosedLoopSlot.kSlot0;
    m_scoreLevel=scoreLevel;

    addRequirements(elevator);
  }

  public ElevatorPosCommand(ElevatorSubsystem elevator,Supplier<ElevatorScoreLevel> scoreLevel,ClosedLoopSlot closedLoopSlot) {
    m_elevator=elevator;
    m_scoreLevel=scoreLevel;
    m_closedLoopSlot=closedLoopSlot;

    addRequirements(elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize(){
    if(m_scoreLevel!=null){
      switch(m_scoreLevel.get()){
        case L1:
        m_pos=ElevatorConstants.l1Pos;
        elevatorlevel = "Level 1";
        break;
        case L2:
        m_pos=ElevatorConstants.l2Pos;
        elevatorlevel = "Level 2";
        break;
        case L3:
        m_pos=ElevatorConstants.l3Pos;
        elevatorlevel = "Level 3";
        break;
        case L4:
        m_pos=ElevatorConstants.l4Pos;
        elevatorlevel = "Level 4";
        break;
        case RETRACT:
        elevatorlevel = "Retract";
        m_pos=ElevatorConstants.elevatorLowLimit;
        break;    
      }
      //notification.setDescription(elevatorlevel);

      //Elastic.sendNotification(notification);

    }else{
      elevatorlevel = "null";
    }

    
  }  

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute(){
    SmartDashboard.putString("Elevator Level", elevatorlevel);
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
