package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.LEDConstants;
import frc.robot.subsystems.ConveyorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.utilities.ADABreakBeam;
import frc.robot.utilities.LED;

public class IntakeConveyCommand extends Command{
  private IntakeSubsystem m_intake;
  private ADABreakBeam m_adaBreakBeam;
  private LED m_led;

  private boolean m_hasNotHitLowBeam=false;

  private enum STATE{
    INTAKING,NOTEINSHOOTPOS
  }

  private STATE m_state=STATE.INTAKING;

  public IntakeConveyCommand(IntakeSubsystem intake,ADABreakBeam adaBreakBeam,LED led){
    m_intake=intake;
    m_adaBreakBeam=adaBreakBeam;
  }

  @Override
  public void end(boolean interrupted){
    m_intake.stopIntake();
  }

  @Override
  public boolean isFinished(){
    boolean returnValue=false;

    switch(m_state){
      case INTAKING:
      m_intake.runIntake(Constants.IntakeConstants.intakeSpeed);

      if(m_adaBreakBeam.getLowBreakBeam()&&!m_hasNotHitLowBeam){
        m_hasNotHitLowBeam=true;
        m_led.SetLEDState(LEDConstants.LEDStates.INTAKELOW);
      }

      if(m_hasNotHitLowBeam&&m_adaBreakBeam.getHighBreakBeam()){
        m_intake.stopIntake();
        m_led.SetLEDState(LEDConstants.LEDStates.INTAKEHASNOTE);
        m_state=STATE.NOTEINSHOOTPOS;
      }
      break;
      case NOTEINSHOOTPOS:
      returnValue=true;
      break;
    }
    return returnValue;
  }
}