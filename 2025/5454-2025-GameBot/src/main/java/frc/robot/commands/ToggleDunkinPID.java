package frc.robot.commands;

import java.util.function.Supplier;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DunkinDonutConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.ElevatorConstants.ElevatorScoreLevel;
import frc.robot.subsystems.DunkinDonutSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

public class ToggleDunkinPID extends Command{
    private DunkinDonutSubsystem m_dunkin;
    private ElevatorSubsystem m_elevator;

    private Supplier<ElevatorScoreLevel> m_scoreLevel;

    private double m_setPoint;

    private boolean m_resetToggle;

    public ToggleDunkinPID(DunkinDonutSubsystem dunkin,Supplier<ElevatorScoreLevel> scoreLevel){
        m_dunkin=dunkin;
        m_scoreLevel=scoreLevel;
    }

    public ToggleDunkinPID(DunkinDonutSubsystem dunkin,ElevatorSubsystem elevator,Supplier<ElevatorScoreLevel> scoreLevel){
        m_dunkin=dunkin;
        m_elevator=elevator;
        m_scoreLevel=scoreLevel;
    }

    public ToggleDunkinPID(DunkinDonutSubsystem dunkin,ElevatorSubsystem elevator,Supplier<ElevatorScoreLevel> scoreLevel,
                                boolean resetToggle){
        m_dunkin=dunkin;
        m_elevator=elevator;
        m_scoreLevel=scoreLevel;
        m_resetToggle=resetToggle;
    }

    @Override
    public void initialize(){
        if(m_resetToggle){
            m_dunkin.resetShouldRunPID();
        }

        if(m_scoreLevel!=null){
            switch(m_scoreLevel.get()){
            case L1:
            m_setPoint=DunkinDonutConstants.l1PosABS;
            break;
            case L2:
            m_setPoint=DunkinDonutConstants.l2PosABS;
            break;
            case L3:
            m_setPoint=DunkinDonutConstants.l3PosABS;
            break;
            case L4:
            m_setPoint=DunkinDonutConstants.l4PosABS;
            break;
            case RETRACT:
            m_setPoint=DunkinDonutConstants.rotateHomePos;
            break;
      }
    }
    }

    @Override
    public boolean isFinished(){
        boolean returnValue=false;

        if(m_elevator.getRelativePos()<ElevatorConstants.aboveTroughPos||m_scoreLevel.get()==ElevatorScoreLevel.RETRACT){
            m_dunkin.toggleLocalPid(m_setPoint);

            if(!m_dunkin.getShouldRunPID()){
                m_dunkin.stop_rotatemotor();
            }

            returnValue=true;
        }
        return returnValue;
    }
}   
