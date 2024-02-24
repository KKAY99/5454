package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ShooterSubsystem;

public class HomeShooterCommand extends CommandBase{
    private ShooterSubsystem m_shooter;
    
    public HomeShooterCommand(ShooterSubsystem shooter){
        m_shooter=shooter;
    }

    @Override
    public void execute(){

    }

    @Override
    public void end(boolean interrupted){}

    @Override
    public boolean isFinished(){
        boolean returnValue=false;

        if(m_shooter.getCanCoderPosition()<Constants.ShooterConstants.homePos){
            m_shooter.RotateShooter(Constants.ShooterConstants.homeRotateSpeed);
        }else{
            m_shooter.RotateShooter(-Constants.ShooterConstants.homeRotateSpeed);
        }

        if(m_shooter.getCanCoderPosition()<=Constants.ShooterConstants.homePos+Constants.ShooterConstants.homeDeadband&&
            m_shooter.getCanCoderPosition()>=Constants.ShooterConstants.homePos-Constants.ShooterConstants.homeDeadband){
                m_shooter.stopRotate();
                m_shooter.zeroRelativePosition();
                returnValue=true;
        }
        return returnValue;
    }
}
