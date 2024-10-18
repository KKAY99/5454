package frc.robot.commands;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.utilities.LED;

public class IntakeConveyCommand extends Command{
  private IntakeSubsystem m_intake;
  private ShooterSubsystem m_shooter;
  private LED m_led;
  private double m_speed;

  private XboxController m_driver;
  private XboxController m_operator;

  private double m_startTime;

  private enum STATE{
    SETANGLE,WAITFORANGLE,INTAKINGNOTE,NOTEINSHOOTPOS
  }

  private STATE m_state=STATE.INTAKINGNOTE;
  public IntakeConveyCommand(IntakeSubsystem intake,ShooterSubsystem shooter,double intakeSpeed,XboxController driver, XboxController operator){
    m_intake=intake;
    m_shooter=shooter;
    //m_led=led;
    m_speed=intakeSpeed;
    m_driver=driver;
    m_operator=operator;
    addRequirements(m_intake);
    addRequirements(m_shooter);
  }

  @Override
  public void initialize(){
    m_state=STATE.SETANGLE;
  }

  @Override
  public void end(boolean interrupted){
    m_intake.stopIntake();
    m_driver.setRumble(RumbleType.kBothRumble,0);
    m_operator.setRumble(RumbleType.kBothRumble,0);
    Logger.recordOutput("Intake/IntakeConveyCommand","Not Running");

  }

  @Override
  public boolean isFinished(){
    boolean returnValue=false;
    double angleGap=0;

    switch(m_state){
      case SETANGLE: 
          m_shooter.setAngle(0);
          m_state=STATE.WAITFORANGLE;
      break;
      case WAITFORANGLE:
        angleGap=Math.abs(m_shooter.getRelativePosition())
                -Math.abs(0);
        if(angleGap<Constants.ShooterConstants.kIntakeFeedAngleDeadband && angleGap>-Constants.ShooterConstants.kIntakeFeedAngleDeadband ){
          m_shooter.stopRotate();
          m_state=STATE.INTAKINGNOTE;
        }
      break;
      case INTAKINGNOTE:
      m_intake.runIntake(m_speed);

      if(m_intake.isBeamBroken()){
        m_intake.stopIntake();
        m_driver.setRumble(RumbleType.kBothRumble,0.95);
        m_operator.setRumble(RumbleType.kBothRumble,0.5);
        m_startTime=Timer.getFPGATimestamp();
        m_state=STATE.NOTEINSHOOTPOS;
      }
      break;
      case NOTEINSHOOTPOS:
      if(Timer.getFPGATimestamp()>(m_startTime+0.3)){
        m_driver.setRumble(RumbleType.kBothRumble,0);
        m_operator.setRumble(RumbleType.kBothRumble,0);
        returnValue=true;
      }
      break;
    }
    Logger.recordOutput("Intake/IntakeConveyCommand",m_state.toString());
    
    return returnValue;
  }
  
}
