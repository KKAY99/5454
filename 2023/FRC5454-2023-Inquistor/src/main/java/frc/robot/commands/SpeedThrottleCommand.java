package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.WPISwerve;
import frc.robot.subsystems.WPIDriveTrainSubsystem;

public class SpeedThrottleCommand extends CommandBase{
    private double m_throttle;
    private WPIDriveTrainSubsystem m_WPIDrive;

    public SpeedThrottleCommand(WPIDriveTrainSubsystem WPIDrive, int throttle){
        m_throttle=throttle;
        m_WPIDrive=WPIDrive;
    }

    @Override
    public void execute(){
        if(m_WPIDrive.speedMultiplier==1.5&&m_throttle==1){

        }else if(m_WPIDrive.speedMultiplier==0.5&&m_throttle==-1){
            
        }else{
            if(m_throttle==-1){
                m_WPIDrive.speedMultiplier=m_WPIDrive.speedMultiplier-0.1;
            }else{
                m_WPIDrive.speedMultiplier=m_WPIDrive.speedMultiplier+0.1;
            }
        }
    }
}
