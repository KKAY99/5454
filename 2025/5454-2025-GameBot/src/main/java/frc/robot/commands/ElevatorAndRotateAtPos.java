package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DunkinDonutConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.ElevatorConstants.ElevatorScoreLevel;
import frc.robot.subsystems.DunkinDonutSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import java.util.function.Supplier;

public class ElevatorAndRotateAtPos extends Command{
    private ElevatorSubsystem m_elevator;
    private DunkinDonutSubsystem m_dunkin;

    private Supplier<ElevatorScoreLevel> m_scoreLevel;

    private double m_rotatePos;
    private double m_elevatorPos;
    
    public ElevatorAndRotateAtPos(ElevatorSubsystem elevator,DunkinDonutSubsystem dunkin,Supplier<ElevatorScoreLevel> scoreLevel){
        m_elevator=elevator;
        addRequirements(elevator);
        m_dunkin=dunkin;
        m_scoreLevel=scoreLevel;
    }

    @Override
    public void initialize(){
        switch(m_scoreLevel.get()){
        case L1:
        m_elevatorPos=ElevatorConstants.l1Pos;
        m_rotatePos=DunkinDonutConstants.l1PosABS;
        break;
        case L2:
        m_elevatorPos=ElevatorConstants.l2Pos;
        m_rotatePos=DunkinDonutConstants.l2PosABS;
        break;
        case L3:
        m_elevatorPos=ElevatorConstants.l3Pos;
        m_rotatePos=DunkinDonutConstants.l3PosABS;
        break;
        case L4:
        m_elevatorPos=ElevatorConstants.l4Pos;
        m_rotatePos=DunkinDonutConstants.l4PosABS;
        break;
        case RETRACT:
        m_elevatorPos=ElevatorConstants.elevatorLowLimit;
        m_rotatePos=DunkinDonutConstants.rotateHomePos;
        break;
        }
    }

    @Override
    public boolean isFinished(){
        boolean returnValue=false;

        if(m_elevator.getRelativePos()>m_elevatorPos-ElevatorConstants.posDeadband&&m_elevator.getRelativePos()<m_elevatorPos+ElevatorConstants.posDeadband&&
            m_dunkin.getAbsoluteEncoderPos()>m_rotatePos-DunkinDonutConstants.posDeadband&&m_dunkin.getAbsoluteEncoderPos()<m_rotatePos+DunkinDonutConstants.posDeadband){
            returnValue=true;
        }


        return returnValue;
    }
}
