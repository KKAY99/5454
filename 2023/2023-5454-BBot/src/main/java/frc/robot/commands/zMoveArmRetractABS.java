package frc.robot.commands;
import frc.robot.Constants;
import frc.robot.subsystems.PneumaticsSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.RotateArmSubsystem;


/** An example command that uses an example subsystem. */
public class zMoveArmRetractABS extends CommandBase {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })

  private final RotateArmSubsystem m_rotate;
  private final PneumaticsSubsystem m_pnuematics;
  private static enum STATE
  {
                  RETRACT,ROTATE,ABORT,END
  }
 private STATE m_state=STATE.RETRACT;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public zMoveArmRetractABS(RotateArmSubsystem rotate,PneumaticsSubsystem pnuematics) {
    m_rotate=rotate;
    m_pnuematics=pnuematics;
  // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_rotate);

}

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_state=STATE.RETRACT;
    System.out.println("Starting Retract");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
   
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
      m_state=STATE.END;
      m_rotate.stop();
      }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    boolean retracted=false;
    boolean returnValue=false;

    switch(m_state){
      case RETRACT:
      retracted = false;

      if(retracted){
        m_state = STATE.ROTATE;
      }
      break;
      case ROTATE:
      retracted=false;
      boolean rotated=false;
      if(m_rotate.getABSPos()<Constants.Rotate.ABSHomePos){
       m_rotate.runWithLimits(Constants.Rotate.rotateAutoInSpeed);
      }else{
             m_rotate.stop();
             rotated=true;
      }
           retracted=true;;
     if(retracted){
       m_state=STATE.END;
     }
     break;
      case ABORT:
      case END:
        System.out.println("Ending Retract Command");
        m_rotate.stop();
        returnValue=true;
    }
    return returnValue;
  }
}
