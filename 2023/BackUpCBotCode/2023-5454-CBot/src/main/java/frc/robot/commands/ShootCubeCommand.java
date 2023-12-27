package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj.Timer;

public class ShootCubeCommand extends CommandBase{
    ShooterSubsystem m_ShooterSubsystem;
    double m_shootTime;
    double m_shootTimeDelay;
    double m_startTime;
    double m_wait;
    double m_shootMotorSpeed;
    
public ShootCubeCommand(ShooterSubsystem shooterSubsystem,double shootMotorSpeed, double wait,double shootTime){
    m_ShooterSubsystem=shooterSubsystem;
    addRequirements(shooterSubsystem);
   
    m_wait=wait;
    m_shootTimeDelay=shootTime;
    m_shootMotorSpeed=shootMotorSpeed;
    
}
public void initialize(){
    m_ShooterSubsystem.spinUpShooterMotors(m_shootMotorSpeed); 
    //set time to allow shooter to actually shoot
    m_startTime = Timer.getFPGATimestamp()+m_wait;
    m_shootTime=m_startTime+m_shootTimeDelay;
 
}
@Override
public boolean isFinished(){
boolean returnValue=false;
double currentTime=Timer.getFPGATimestamp();
//if the wait to shoot time passed
System.out.println(currentTime +" " + m_startTime +" " + m_shootTime);
if(currentTime>m_startTime){
    //if the wait to finish shooting time is passed then stop shooting
    if(currentTime>m_shootTime){
        m_ShooterSubsystem.stop();

        returnValue=true;
    }else // if shoot time hasn't passed just keep/start shooting
    {
    
        m_ShooterSubsystem.expellCube(m_shootMotorSpeed);
        returnValue=false;
    }
}
return returnValue;
}
@Override
public void end(boolean interrupted) {

    m_ShooterSubsystem.stop();
}

@Override
public void execute(){
 
  

}
}