package frc.robot.commands;
import frc.robot.Constants;
import frc.robot.Constants.Pneumatics;
import frc.robot.subsystems.RotateArmSubsystem;
import frc.robot.subsystems.PneumaticsSubsystem;
import edu.wpi.first.util.concurrent.Event;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.classes.Limelight;



/** An example command that uses an example subsystem. */
public class zMoveArmExtendABS extends CommandBase {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  
  private final RotateArmSubsystem m_rotate;
  private final PneumaticsSubsystem m_pnuematics;
  private Constants.TargetHeight m_targetLevel;
  private double m_posInitLift;
  private double m_posFullLiftStage1;
  private double m_posFullLiftStage2;
  private double m_angleStage1ABS;
  private double m_angleStage2ABS;
  private static int kTolerance= 5;
  private Limelight m_limelight;
  private boolean m_checkForTarget;
  private boolean m_openClaw;
  private int targetChecks=0;
  private final int kCheckTargetTimes=10;
  private static enum STATE
  {
                  CANSEETARGET,EXTEND,ROTATE,OPENCLAW,ABORT,END
  }
 private STATE m_state=STATE.EXTEND;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public zMoveArmExtendABS(RotateArmSubsystem rotate,PneumaticsSubsystem pnuematics, Limelight limelight, Constants.TargetHeight targetLevel, boolean checkForTarget,boolean openClaw) {
    m_rotate = rotate;
    m_pnuematics = pnuematics;
    m_limelight = limelight;
    m_checkForTarget = checkForTarget;
    m_targetLevel=targetLevel;
    m_openClaw=openClaw;
  // Use addRequirements() here to declare subsystem dependencies.
      addRequirements(m_rotate);  
      switch(targetLevel){
      case MIDDLECONE:
        m_angleStage1ABS=Constants.Rotate.angleMiddleConeABS;
      break;
      case BOTTOMCONE:
        m_angleStage1ABS=m_rotate.getABSPos();
        m_angleStage2ABS=m_rotate.getABSPos();
        break;
      case MIDDLECUBE:
        m_angleStage1ABS=Constants.Rotate.angleMiddleCubeABS;
      break;
      case BOTTOMCUBE:
        m_angleStage1ABS=m_rotate.getABSPos();
        m_angleStage2ABS=m_rotate.getABSPos();
        break;
    }
     
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_state=STATE.CANSEETARGET;
    targetChecks=0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  //System.out.println("Move Arm Extend " + m_state.toString());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
      System.out.println("Ending Auto Score");
      m_state=STATE.END;
      m_rotate.stop();
      }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    
    boolean returnValue=false;
    System.out.println(m_state);
    switch(m_state){
      case CANSEETARGET:
      //if checking for target use Limelight to check otherwise move to Extend
      if(m_checkForTarget){
        m_limelight.update(); /// force limelight to update
        if(m_limelight.isTargetAvailible()){
          m_state = STATE.EXTEND;
        }else{  
          if(targetChecks<kCheckTargetTimes){
              targetChecks++;
              System.out.println("Target Not Visibile Check #" + targetChecks );
              m_state=STATE.CANSEETARGET;
          }
          else{
             m_state = STATE.ABORT;
          }
        }
      }else{
        m_state = STATE.EXTEND;
      }
      break;
      case EXTEND:
          m_pnuematics.setExtensionSolenoid(true);
          m_state=STATE.ROTATE;
          returnValue=false;

          break;
      case ROTATE:
         if(m_rotate.getRotatePos()<m_angleStage2ABS){
          m_rotate.runWithLimits(Constants.Rotate.rotateAutoOutStage2Speed);
         }else{
          System.out.println("Rotate ABS on Stop - " + m_rotate.getRotatePos());
          m_rotate.stop();
         }
         returnValue=false;
         m_state=STATE.OPENCLAW;
        break;

      case OPENCLAW:
         m_pnuematics.setBottomClawSolenoid(true);
         m_pnuematics.setTopClawSolenoid(true);
         m_state=STATE.END;
         break;
      case ABORT:
         System.out.println("Aborting Auto Score");
      case END:
      System.out.println("Ending Auto Score");
      m_rotate.stop();
      returnValue=true;
  }
  return returnValue;
 }
}

