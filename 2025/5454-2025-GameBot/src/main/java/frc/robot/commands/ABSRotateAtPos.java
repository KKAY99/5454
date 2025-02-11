package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DunkinDonutConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.ElevatorConstants.ElevatorScoreLevel;
import frc.robot.subsystems.DunkinDonutSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import java.util.function.Supplier;

public class ABSRotateAtPos extends Command{
    private DunkinDonutSubsystem m_dunkin;

    private Supplier<ElevatorScoreLevel> m_scoreLevel;

    private double m_rotatePos;
    
    public ABSRotateAtPos(DunkinDonutSubsystem dunkin,Supplier<ElevatorScoreLevel> scoreLevel){
        m_dunkin=dunkin;
        m_scoreLevel=scoreLevel;
    }

    @Override
    public void initialize(){
        switch(m_scoreLevel.get()){
        case L1:
        m_rotatePos=DunkinDonutConstants.l1PosABS;
        break;
        case L2:
        m_rotatePos=DunkinDonutConstants.l2PosABS;
        break;
        case L3:
        m_rotatePos=DunkinDonutConstants.l3PosABS;
        break;
        case L4:
        m_rotatePos=DunkinDonutConstants.l4PosABS;
        break;
        case RETRACT:
        m_rotatePos=DunkinDonutConstants.rotateHomePos;
        break;
        }
    }

    @Override
    public boolean isFinished(){
        boolean returnValue=false;

        if(m_dunkin.getAbsoluteEncoderPos()>m_rotatePos-DunkinDonutConstants.posDeadband&&m_dunkin.getAbsoluteEncoderPos()<m_rotatePos+DunkinDonutConstants.posDeadband){
            returnValue=true;
        }


        return returnValue;
    }
}
