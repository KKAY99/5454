package frc.robot.commands;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class SmartShootManualAimCommand extends Command {
  private ShooterSubsystem m_shooter;
  private double m_power;

  private double m_startTime;
  private double kTimeToRun=Constants.Shooter.feederTimeToRun;
  private double m_angle;
  private boolean m_useAngle;
  private boolean m_useLimeLight;
  private enum States{
    STARTMOTORS,WAITFORVELOC,SHOOT,STOP,END
  }

  private States m_currentState=States.STARTMOTORS;

  public SmartShootManualAimCommand(ShooterSubsystem shooter,double power) {
    m_shooter = shooter;
    m_power = power;
  }

 // Called when the command is initially scheduled.
 @Override
 public void initialize() {
    m_currentState=States.STARTMOTORS;
 }
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooter.stopShooterMotors();
    m_shooter.stopFeeder();
    m_shooter.stopShooterIncline();
    m_currentState=States.STARTMOTORS;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    boolean returnValue=false;
    System.out.println(m_currentState.toString());
    switch(m_currentState){
      case STARTMOTORS:
        m_shooter.runShooterMotors(m_power);
        m_currentState=States.WAITFORVELOC;      
        
      break;
      case WAITFORVELOC:
        if(m_shooter.isAtVeloc(m_power)){
          m_startTime=Timer.getFPGATimestamp();
          m_currentState=States.SHOOT;
        }
      break;
      case SHOOT:
        
        m_shooter.runFeeder(Constants.Shooter.ShooterFeederSpeed);

        if(Timer.getFPGATimestamp()>m_startTime+kTimeToRun){
          m_currentState=States.STOP;
        }
      break;
      case STOP:
        m_shooter.stopFeeder();
        m_currentState=States.END;
      break;
      case END:
      returnValue=true;
      break;
    }
    return returnValue;
  }
}
