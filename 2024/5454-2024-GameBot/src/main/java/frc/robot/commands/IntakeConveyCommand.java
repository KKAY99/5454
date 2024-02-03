package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ConveyorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.utilities.ADABreakBeam;

public class IntakeConveyCommand extends Command{
  private IntakeSubsystem m_intake;
  private ConveyorSubsystem m_convey;
  private ADABreakBeam m_adaBreakBeam;

  private enum STATE{
    INTAKING,CONVEYING,NOTEINSHOOTPOS
  }

  private STATE m_state=STATE.INTAKING;

  public IntakeConveyCommand(IntakeSubsystem intake,ConveyorSubsystem convey,ADABreakBeam adaBreakBeam){
    m_convey=convey;
    m_intake=intake;
    m_adaBreakBeam=adaBreakBeam;
  }

  @Override
  public void end(boolean interrupted){
    m_intake.stopIntake();
    m_convey.stopConveyer();
  }

  @Override
  public boolean isFinished(){
    boolean returnValue=false;

    switch(m_state){
      case INTAKING:
      m_intake.runIntake(Constants.IntakeConstants.intakeSpeed);

      if(m_adaBreakBeam.getLowBreakBeam()||m_adaBreakBeam.getHighBreakBeam()){
        m_intake.stopIntake();
        m_state=STATE.CONVEYING;
      }

      break;
      case CONVEYING:
      m_convey.runConveyor(Constants.ConveyerConstants.conveySpeed);

      if(m_adaBreakBeam.getLowBreakBeam()){
        m_convey.stopConveyer();
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
