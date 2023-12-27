package frc.robot.commands;
import frc.robot.Constants;
import frc.robot.Constants.Lift;
import frc.robot.Constants.Pneumatics;
import frc.robot.Constants.TargetHeight;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.RotateArmSubsystem;
import frc.robot.subsystems.PnuematicsSubystem;
import edu.wpi.first.util.concurrent.Event;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.classes.Limelight;
import frc.robot.commands.FloorIntakeCommand.STATE;
import frc.robot.subsystems.ClawSubsystem;
import edu.wpi.first.wpilibj.Timer;



/** An example command that uses an example subsystem. */
public class zMoveArmExtendABS extends CommandBase {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  
  private final ElevatorSubsystem m_elevator;
  private final RotateArmSubsystem m_rotate;
  private final PnuematicsSubystem m_pPnuematicsSubystem;
  private final ClawSubsystem m_claw;
  private double m_posInitLift;
  private double m_posFullLiftStage1;
  private double m_posFullLiftStage2;
  private double m_angleStage1ABS;
  private Constants.TargetHeight m_targetLevel;
  private double m_angleStage2ABS;
  private static int kTolerance= 5;
  private Limelight m_limelight;
  private boolean m_checkForTarget;
  private boolean m_shouldRotateClaw;
  private boolean m_openClaw;
  private int targetChecks=0;
  private final int kCheckTargetTimes=10;
  private double kClawRunTime=0.5;
  private boolean m_hasRun=false;
  private double m_endTime;

  private static enum STATE
  {
                  CANSEETARGET,INITLIFT,ROTATE,EXTENDANDROTATE,EXTENDLIFT,FINALROTATE,OPENCLAW,ABORT,END
  }
 private STATE m_state=STATE.INITLIFT;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public zMoveArmExtendABS(ElevatorSubsystem elevator, RotateArmSubsystem rotate,PnuematicsSubystem pnumatics, ClawSubsystem claw, Limelight limelight, Constants.TargetHeight targetLevel, boolean checkForTarget, boolean shouldRotateClaw,boolean openClaw) {
    m_elevator = elevator;
    m_rotate = rotate;
    m_pPnuematicsSubystem=pnumatics;
    m_claw = claw;
    m_limelight = limelight;
    m_checkForTarget = checkForTarget;
    m_targetLevel=targetLevel;
    m_shouldRotateClaw = shouldRotateClaw;
    m_openClaw=openClaw;
  // Use addRequirements() here to declare subsystem dependencies.
      addRequirements(m_elevator);
      addRequirements(m_rotate);  
      switch(targetLevel){
      case TOPCONE:
        m_posFullLiftStage1=Constants.Lift.posHighConeFullLiftStage1;
        m_posFullLiftStage2=Constants.Lift.posHighConeFullLiftStage2;
        m_angleStage1ABS=Constants.Rotate.angleHighConeStage1ABS;
        m_angleStage2ABS=Constants.Rotate.angleHighConeStage2ABS;
      break;
      case MIDDLECONE:
        m_posFullLiftStage1=Constants.Lift.posMiddleConeFullLiftStage1;
        m_posFullLiftStage2=Constants.Lift.posMiddleConeFullLiftStage2;
        m_angleStage1ABS=Constants.Rotate.angleMiddleConeStage1ABS;
        m_angleStage2ABS=Constants.Rotate.angleMiddleConeStage2ABS;
      break;
      case BOTTOMCONE:
        m_posFullLiftStage1=Constants.Lift.posLowConeFullLiftStage1;
        m_posFullLiftStage2=Constants.Lift.posLowConeFullLiftStage2;
        m_angleStage1ABS=m_rotate.getAbsolutePos();
        m_angleStage2ABS=m_rotate.getAbsolutePos();
        break;
      case TOPCUBE:
      m_posFullLiftStage1=Constants.Lift.posHighCubeFullLiftStage1;
      m_posFullLiftStage2=Constants.Lift.posHighCubeFullLiftStage2;
      m_angleStage1ABS=Constants.Rotate.angleHighCubeStage1ABS;
      m_angleStage2ABS=Constants.Rotate.angleHighCubeStage2ABS;
      break;
      case MIDDLECUBE:
        m_posFullLiftStage1=Constants.Lift.posMiddleCubeFullLiftStage1;
        m_posFullLiftStage2=Constants.Lift.posMiddleCubeFullLiftStage2;
        m_angleStage1ABS=Constants.Rotate.angleMiddleCubeStage1ABS;
        m_angleStage2ABS=Constants.Rotate.angleMiddleCubeStage2ABS;
      break;
      case BOTTOMCUBE:
        m_posFullLiftStage1=Constants.Lift.posLowCubeFullLiftStage1;
        m_posFullLiftStage2=Constants.Lift.posLowCubeFullLiftStage2;
        m_angleStage1ABS=m_rotate.getAbsolutePos();
        m_angleStage2ABS=m_rotate.getAbsolutePos();
        break;
      case PLAYERSTATION:
        m_posFullLiftStage1 = Constants.Lift.posPlayerLiftStage1;
        m_posFullLiftStage2 = Constants.Lift.posPlayerLiftStage2;
        m_angleStage1ABS = Constants.Rotate.anglePlayerStage1ABS;
        m_angleStage2ABS = Constants.Rotate.anglePlayerStage2ABS;
        break;
      case SLIDE:
        m_posFullLiftStage1 = Constants.Lift.posSlideStage1;
        m_posFullLiftStage2 = Constants.Lift.posSlideStage2;
        m_angleStage1ABS = Constants.Rotate.angleSlideStage1ABS;
        m_angleStage2ABS = Constants.Rotate.angleSlideStage2ABS;
       break;
    }
    m_posInitLift=Constants.Lift.posInitLift;
     
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
      m_elevator.stop();
      m_rotate.stopRotate();
      }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    
    boolean returnValue=false;
    System.out.println(m_state);
    switch(m_state){
      case CANSEETARGET:
      //if checking for target use Limelight to check otherwise move to INITLIFT
      if(m_checkForTarget){
        m_limelight.update(); /// force limelight to update
        if(m_limelight.isTargetAvailible()){
          m_state = STATE.INITLIFT;
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
        m_state = STATE.INITLIFT;
      }
      break;
      case INITLIFT:
      m_pPnuematicsSubystem.setConveyorPunch(true);

      // Encoder is negative as it lifts up
          if(m_elevator.getElevatorPos()>m_posInitLift){
            m_elevator.runWithOutLimit(Constants.Lift.liftAutoExtendStage1Speed);
          }else{
            m_elevator.stop();
            m_state=STATE.EXTENDANDROTATE;
          // m_state=STATE.ABORT;
          }
          returnValue=false;
          break;
      case ROTATE:
         if(m_rotate.getAbsolutePos()<m_angleStage2ABS){
          m_rotate.rotate(Constants.Rotate.rotateAutoOutStage2Speed);
         } else{
          System.out.println("Rotate ABS on Stop - " + m_rotate.getAbsolutePos());
          m_rotate.stopRotate();
          m_state=STATE.EXTENDLIFT;
         }
         returnValue=false;
          break;
      case EXTENDLIFT:
       // Encoder is negative as it lifts up
       if(m_elevator.getElevatorPos()>m_posFullLiftStage2){
        m_elevator.runWithOutLimit(Constants.Lift.liftAutoExtendStage2Speed);
      } else{
        if(m_elevator.getElevatorPos()<m_posFullLiftStage2){
          m_elevator.runWithOutLimit(-Constants.Lift.liftAutoExtendStage2Speed);
        }else{
          m_elevator.stop();
          m_state=STATE.END;
        }
      }
     
        returnValue=false;
        break;
      case EXTENDANDROTATE:
       boolean extended=false;
       boolean rotated=false;
       double currentRotateEncoder=m_rotate.getAbsolutePos();
       System.out.println("Current Rotate" + currentRotateEncoder);
      if(currentRotateEncoder>m_angleStage1ABS){
        m_rotate.rotate(Constants.Rotate.rotateAutoOutStage1Speed);
       } else{
          if(currentRotateEncoder>m_angleStage2ABS){
            m_rotate.rotate(Constants.Rotate.rotateAutoOutStage2Speed);  
            } else{
              m_rotate.stopRotate();
              rotated=true;
             }
      }
      
       if(m_elevator.getElevatorPos()>m_posFullLiftStage1){
        m_elevator.runWithOutLimit(Constants.Lift.liftAutoExtendStage1Speed);
       }else{
          if(m_elevator.getElevatorPos()>m_posFullLiftStage2){
            m_elevator.runWithOutLimit(Constants.Lift.liftAutoExtendStage2Speed);
          }else{
            if(m_elevator.getElevatorPos()<m_posFullLiftStage2){
              System.out.println("Past Target");
              if(Math.abs(m_elevator.getElevatorPos()-m_posFullLiftStage2) > kTolerance){
                System.out.println("Running Back");
                m_elevator.runWithOutLimit(-Constants.Lift.liftAutoExtendStage2Speed);
              } else{
                  System.out.println("Past Target but within Tolerance" + m_elevator.getElevatorPos());
                  m_elevator.stop();
                  extended=true;
              }
            }else{
              System.out.println("Stopping 1");
              m_elevator.stop();
            }    
          }
        }

      if(rotated && extended){
        System.out.println("rotated && extended");
        if(m_openClaw){
              m_state=STATE.OPENCLAW;
        }else{
          System.out.println("Ending Auto Score - Not Opening Claw");
          m_state=STATE.END;
        }
      }
      //System.out.println("Rotate " + rotated + "Elevator " + extended);
      break;
      case FINALROTATE:
      //kk 3/4 commented out for playoffs
      m_state=STATE.END;
      /*  if(m_targetLevel==Constants.TargetHeight.PLAYERSTATION){
        m_state=STATE.END;
       }else{
        if(m_rotate.getAbsolutePos() > m_angleStage2ABS-0.05){
          m_rotate.rotate(Constants.Rotate.rotateAutoOutStage2Speed);
        }else {
          m_rotate.stopRotate();
          m_state=STATE.END;
        }
       }*/
      break;
      case OPENCLAW:
      if(m_openClaw){
        if(m_hasRun==false){
          //pivot down
          //m_pPnuematicsSubystem.setClaw(false);
          m_endTime=Timer.getFPGATimestamp()+kClawRunTime; 
          m_claw.runClaw(0.3); //was .5
          m_hasRun=true;
         }

         if(returnValue=(m_endTime<Timer.getFPGATimestamp()&&m_hasRun)){
          m_claw.stopClaw();
          m_hasRun=false;
          m_state=STATE.END;
         }
      }else{
        m_state=STATE.END;
      }
         break;
      case ABORT:
         System.out.println("Aborting Auto Score");
      case END:
      System.out.println("Ending Auto Score");
        //m_elevator.SetPosAndMove(m_posFullLiftStage2);
        m_hasRun=false;
        m_elevator.stop();
        m_rotate.stopRotate();
        returnValue=true;

    }
    
    return returnValue;
  }
}
