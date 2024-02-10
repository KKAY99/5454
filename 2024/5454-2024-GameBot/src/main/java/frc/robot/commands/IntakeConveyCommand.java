package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.LEDConstants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.utilities.ADABreakBeam;
import frc.robot.utilities.LED;
import frc.robot.utilities.Lasercan;

public class IntakeConveyCommand extends Command{
  private IntakeSubsystem m_intake;
  private Lasercan m_laserCan;
  private LED m_led;

  private boolean m_hasHitLowBeam=false;

  private enum STATE{
    INTAKING,NOTEINSHOOTPOS
  }

  private STATE m_state=STATE.INTAKING;

  public IntakeConveyCommand(IntakeSubsystem intake,Lasercan lasercan,LED led){
    m_intake=intake;
    m_laserCan=lasercan;

    addRequirements(m_intake);
  }

  @Override
  public void end(boolean interrupted){
    m_hasHitLowBeam=false;
    m_intake.stopIntake();
  }

  @Override
  public boolean isFinished(){
    boolean returnValue=false;
    System.out.println("Lasercan BreakBeam: "+m_laserCan.LowTurretBreakBeam());
    System.out.println("Lasercan Distance: "+m_laserCan.GetDistanceInMMLow());

    switch(m_state){
      case INTAKING:
      m_intake.runIntake(Constants.IntakeConstants.intakeSpeed);

      if(m_laserCan.LowTurretBreakBeam()&&!m_hasHitLowBeam){
        m_hasHitLowBeam=true;
        //m_led.SetLEDState(LEDConstants.LEDStates.INTAKELOW);
      }

      if(m_hasHitLowBeam&&m_laserCan.LowTurretBreakBeam()){
        m_intake.stopIntake();
        //m_led.SetLEDState(LEDConstants.LEDStates.INTAKEHASNOTE);
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
