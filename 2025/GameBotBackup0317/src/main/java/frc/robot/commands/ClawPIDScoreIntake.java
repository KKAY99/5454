package frc.robot.commands;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.spark.ClosedLoopSlot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DunkinDonutConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.ElevatorConstants.ElevatorScoreLevel;
import frc.robot.subsystems.DunkinDonutSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

public class ClawPIDScoreIntake extends Command{
    private DunkinDonutSubsystem m_dunkin;
    private ElevatorSubsystem m_elevator;

    private Supplier<ElevatorScoreLevel> m_scoreLevel;

    private double m_setPoint;
    private double m_elevDesiredPos;
    private double m_speed;
    private double m_elevResetPos;
    private double m_clawResetPos;

    private boolean m_isRunning;

    private enum States{
        SETELEVPOS,WAITFORELEVPOS,SETPID,WAITFORPID,RUNALGAE,END
    }

    private States m_currentState=States.SETELEVPOS;

    public ClawPIDScoreIntake(DunkinDonutSubsystem dunkin,ElevatorSubsystem elevator,double elevDesiredPos,
                        double setPoint,double speed,double elevResetPos,double clawResetPos){
        m_dunkin=dunkin;
        m_elevator=elevator;

        m_setPoint=setPoint;
        m_elevDesiredPos=elevDesiredPos;
        m_speed=speed;
        m_clawResetPos=clawResetPos;
        m_elevResetPos=elevResetPos;
    }

    @Override
    public void initialize(){
        m_currentState=States.SETELEVPOS;
        m_isRunning=true;
        m_dunkin.resetShouldRunPID();
    }

    @Override
    public void end(boolean interrupted) {
        m_dunkin.stopAlgeaMotor();
        m_dunkin.resetShouldRunPID();
        m_dunkin.toggleLocalPid(m_clawResetPos);
        m_elevator.set_referance(m_elevResetPos,ClosedLoopSlot.kSlot1);
        m_isRunning=false;
    }

    @Override
    public boolean isFinished(){
        boolean returnValue=false;

        switch(m_currentState){
        case SETELEVPOS:
            m_elevator.set_referance(m_elevDesiredPos,ClosedLoopSlot.kSlot1);

            m_currentState=States.WAITFORELEVPOS;
        break;
        case WAITFORELEVPOS:
            if(m_elevator.getRelativePos()>m_elevDesiredPos-ElevatorConstants.posDeadband&&
                    m_elevator.getRelativePos()<m_elevDesiredPos+ElevatorConstants.posDeadband){
                m_currentState=States.SETPID;
            }
        break;
        case SETPID:
            m_dunkin.toggleLocalPid(m_setPoint);

            m_currentState=States.WAITFORPID;
        break;
        case WAITFORPID:
            if(m_dunkin.getAbsoluteEncoderPos()>m_setPoint-DunkinDonutConstants.posDeadband&&
                    m_dunkin.getAbsoluteEncoderPos()<m_setPoint+DunkinDonutConstants.posDeadband){
                m_currentState=States.RUNALGAE;
            }
        break;
        case RUNALGAE:
            m_dunkin.runAlgaeMotor(m_speed);
        break;
        case END:
            returnValue=true;
        }

        Logger.recordOutput("Commands/ClawPIDScoreIntake/IsRunning",m_isRunning);
        Logger.recordOutput("Commands/ClawPIDScoreIntake/CurrentState",m_currentState);

        return returnValue;
    }
}   
