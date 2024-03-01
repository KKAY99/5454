package frc.robot.commands;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.utilities.LED;

public class IntakeConveyCommand extends Command{
  private IntakeSubsystem m_intake;
  private LED m_led;
  private double m_speed;

  private enum STATE{
    INTAKINGNOTE,HASNOTE,NOTEINSHOOTPOS
  }

  private STATE m_state=STATE.INTAKINGNOTE;
  public IntakeConveyCommand(IntakeSubsystem intake,double intakeSpeed, LED led){
    m_intake=intake;
    m_led=led;
    m_speed=intakeSpeed;
    addRequirements(m_intake);
  }

  @Override
  public void initialize(){
    m_state=STATE.INTAKINGNOTE;
  }

  @Override
  public void end(boolean interrupted){
    m_intake.stopIntake();
    Logger.recordOutput("Intake/IntakeConveyCommand","Not Running");

  }

  @Override
  public boolean isFinished(){
    boolean returnValue=false;

    switch(m_state){
      case INTAKINGNOTE:
      m_intake.runIntake(m_speed);

      if(m_intake.isBeamBroken()&&m_intake.checkBreakBeamValue()){
      //  m_led.SetLEDState(LEDConstants .INTAKELOW);
        m_intake.stopIntake();
        m_state=STATE.NOTEINSHOOTPOS;
      }
      break;
      case NOTEINSHOOTPOS:
      returnValue=true;
      break;
    }
    Logger.recordOutput("Intake/IntakeConveyCommand",m_state.toString());
    
    return returnValue;
  }
  
}
