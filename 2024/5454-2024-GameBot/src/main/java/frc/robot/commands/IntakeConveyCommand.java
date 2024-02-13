package frc.robot.commands;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.LEDConstants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.utilities.ADABreakBeam;
import frc.robot.utilities.LED;
import frc.robot.utilities.Lasercan;
import frc.robot.utilities.AnalogAutoDirectFB6DN0E;
public class IntakeConveyCommand extends Command{
  private IntakeSubsystem m_intake;
  private AnalogAutoDirectFB6DN0E m_breakBeam;
  private LED m_led;

  private enum STATE{
    INTAKINGNOTE,HASNOTE,NOTEINSHOOTPOS
  }

  private STATE m_state=STATE.INTAKINGNOTE;
  private boolean m_isRunning;
  public IntakeConveyCommand(IntakeSubsystem intake, int breakbeamport,LED led){
    m_intake=intake;
    m_breakBeam = new AnalogAutoDirectFB6DN0E(breakbeamport);
    //m_laserCan=lasercan;
    m_isRunning=false;
    addRequirements(m_intake);
  }

  @Override
  public void initialize(){
    m_state=STATE.INTAKINGNOTE;
    m_isRunning=true;
  }

  @Override
  public void end(boolean interrupted){
    m_intake.stopIntake();
    m_isRunning=false;
    Logger.recordOutput("Intake/IntakeConveyCommand","Not Running");

  }

  @Override
  public boolean isFinished(){
    boolean returnValue=false;
    //System.out.println("Lasercan BreakBeam: "+m_laserCan.HighTurretBreakBeam());
    //System.out.println("Lasercan Distance: "+m_laserCan.GetDistanceInMMHigh());

    switch(m_state){
      case INTAKINGNOTE:
      m_intake.runIntake(Constants.IntakeConstants.intakeConveyGetNoteSpeed);

      if(m_breakBeam.isBeamBroken()){
        //m_led.SetLEDState(LEDConstants.LEDStates.INTAKELOW);
        //m_intake.stopIntake();
        m_state=STATE.NOTEINSHOOTPOS;
      }
      break;
      case HASNOTE:
      m_intake.runIntake(Constants.IntakeConstants.intakeConveyHasNoteSpeed);

      /*if(m_laserCan.HighTurretBreakBeam()){
        m_intake.stopIntake();
        m_state=STATE.NOTEINSHOOTPOS;
      }*/
      break;
      case NOTEINSHOOTPOS:
      returnValue=true;
      break;
    }
    Logger.recordOutput("Intake/IntakeConveyCommand",m_state.toString());
    Logger.recordOutput("Intake/BreakBeamStatus",m_breakBeam.isBeamBroken());
    Logger.recordOutput("Intake/BreakBeamValue",m_breakBeam.getRawValue());

    return returnValue;
  }
  
}
