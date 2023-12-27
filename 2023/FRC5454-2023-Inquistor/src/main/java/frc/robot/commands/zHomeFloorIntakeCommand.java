package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.FloorIntakeSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;

public class zHomeFloorIntakeCommand extends CommandBase{
    private FloorIntakeSubsystem m_intake;
    private ElevatorSubsystem m_lift;
    private double m_timeOut;
    private double m_speed;
    private double m_startTime;
    private double m_liftHeight;
    public zHomeFloorIntakeCommand(FloorIntakeSubsystem intake,ElevatorSubsystem lift,double speed,double liftHeight,double timeOut){
        m_intake = intake;
        m_speed = speed;
        m_lift=lift;
        m_liftHeight=liftHeight;
        m_timeOut = timeOut;
    }

    @Override
    public void initialize(){
        m_startTime=Timer.getFPGATimestamp();
        m_lift.SetPosAndMove(m_liftHeight-10);
    }
    
    private void homeFloorintake(){
        m_intake.resetEncodertoZero();
        m_intake.stopRotate();
    }
    
    @Override 
    public boolean isFinished(){
        double currentTime=Timer.getFPGATimestamp();
        if(currentTime>m_startTime+m_timeOut){
            homeFloorintake();
            return true;
        }else{
            if(m_intake.getLimitSwitch()){
               homeFloorintake();
                return true;
            }else{
                if(m_lift.getElevatorPos() <= m_liftHeight){
                        m_intake.rotate(m_speed);
                }
            }

        }
        return false;
    }
}
