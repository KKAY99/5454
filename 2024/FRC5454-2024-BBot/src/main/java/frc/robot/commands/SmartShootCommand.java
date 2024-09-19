package frc.robot.commands;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class SmartShootCommand extends Command {
  private ShooterSubsystem m_shooter;
  private double m_power;

  private double m_startTime;
  private double kTimeToRun=Constants.Shooter.feederTimeToRun;
  private double m_angle;
  private boolean m_useAngle;
  private boolean m_useLimeLight;
  private enum States{
    STARTMOTORS,VISIONGETANGLE,SETANGLE,WAITFORANGLE,WAITFORVELOC,SHOOT,LOWER,WAITFORSHOOTERLOWER,END
  }

  private States m_currentState=States.STARTMOTORS;

  public SmartShootCommand(ShooterSubsystem shooter,double power, boolean useLimeLight, boolean setAngle, double angle) {
    m_shooter = shooter;
    m_power = power;
    m_useAngle=setAngle;
    m_angle=angle;
    m_useLimeLight=useLimeLight;
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
        if(m_useLimeLight == true){
          m_currentState=States.VISIONGETANGLE;
        }else if(m_useAngle == true){
          m_currentState=States.SETANGLE;
        }else{
          m_currentState=States.WAITFORVELOC;
        }
        
        
        
      break;
      case VISIONGETANGLE:

        //get current angle target from limelight and set m_angle to it

        m_currentState=States.SETANGLE;
      break;
      case SETANGLE:
        //m_shooter.SetReference(m_angle);
        m_shooter.gotoPosition(m_angle);
        m_currentState=States.WAITFORANGLE;
        break;
      case WAITFORANGLE:
        if(m_shooter.isAtAngleWithDeadband(m_angle)){
          m_shooter.stopShooterIncline();
          m_currentState=States.WAITFORVELOC;
         
        }
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
          m_currentState=States.LOWER;
        }
      break;
      case LOWER:
      m_shooter.SetReference(Constants.Shooter.shooterInclinePosLow);
      m_currentState=States.WAITFORSHOOTERLOWER;
      case WAITFORSHOOTERLOWER:
      if(m_shooter.isAtAngleWithDeadband(Constants.Shooter.shooterInclinePosLow)){
        m_shooter.stopShooterIncline();
        m_currentState=States.END;
      }
      case END:
      returnValue=true;
      break;
    }
    return returnValue;
  }
}
