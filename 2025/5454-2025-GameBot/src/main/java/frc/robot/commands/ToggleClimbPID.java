package frc.robot.commands;

import java.util.function.Supplier;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DunkinDonutConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.ElevatorConstants.ElevatorScoreLevel;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.DunkinDonutSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

public class ToggleClimbPID extends Command{
    private ClimbSubsystem m_climb;

    private double m_setPoint;

    public ToggleClimbPID(ClimbSubsystem climb,double setPoint){
        m_climb=climb;
        m_setPoint=setPoint;
    }

    @Override
    public boolean isFinished(){
        m_climb.togglePID(m_setPoint);

        if(!m_climb.getPIDToggle()){
            m_climb.stop();;
        }
        return true;
    }
}   
