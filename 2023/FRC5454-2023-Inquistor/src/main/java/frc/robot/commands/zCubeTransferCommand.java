package frc.robot.commands;
import frc.robot.Constants;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.FloorIntakeSubsystem;
import frc.robot.subsystems.PnuematicsSubystem;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.RotateArmSubsystem;


/** An example command that uses an example subsystem. */
public class zCubeTransferCommand extends CommandBase {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  
  private final ElevatorSubsystem m_elevator;
  private final RotateArmSubsystem m_rotate;
  private final PnuematicsSubystem m_pnuematics;
  private final FloorIntakeSubsystem m_floorIntake;
  private final double kFloorIntakeRotateTolerance=.2;
  private final double kPushCubeTime=0.5;
  private final double kPushCubeSpeed=0.2;
  private static enum STATE
  {
                  MOVELIFT,ROTATEINTAKE,OPENCLAW,LOWERLIFT,PUSHCUBE,CLOSECLAW,ABORT,END
  }
 private STATE m_state=STATE.MOVELIFT;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public zCubeTransferCommand (ElevatorSubsystem elevator, RotateArmSubsystem rotate,PnuematicsSubystem pnuematics,FloorIntakeSubsystem floor) {
    m_elevator=elevator;
    m_rotate=rotate;
    m_pnuematics=pnuematics;
    m_floorIntake=floor;
  // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_elevator);
    addRequirements(m_rotate);

}

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_state=STATE.MOVELIFT;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
   
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
      m_state=STATE.END;
      m_elevator.stop();
      m_rotate.stopRotate();
      m_floorIntake.stopIntake();
      m_floorIntake.stopRotate();
      }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    boolean returnValue=false;
    System.out.println(m_state);
    switch(m_state){
      case MOVELIFT:
      boolean lifted=false;
      lifted = false;
      if(m_elevator.getElevatorPos()>Constants.Lift.posLiftOutfIntake){
        m_elevator.runWithOutLimit(Constants.Lift.liftAutoRetract1Speed);
       }else{     
          lifted = true;
       }

       if(lifted){
        m_state = STATE.ROTATEINTAKE;
       }
      break;
      case ROTATEINTAKE:
      boolean intakeRotated=false;
      double rotateDistance=Constants.FloorIntake.rotateHandOff;
       if(Math.abs(rotateDistance)<kFloorIntakeRotateTolerance){
          //in position 
          intakeRotated=true;
          m_rotate.stopRotate();
       } else{
        if(rotateDistance>0){
          m_rotate.rotate(Constants.FloorIntake.intakeRotateSpeed);
        }else {
          m_rotate.rotate(-Constants.FloorIntake.intakeRotateSpeed);
        }
  
       }
       if(intakeRotated){
          m_state=STATE.OPENCLAW;

       }
     break;
     case OPENCLAW:
     m_pnuematics.setClaw(true);
     m_state=STATE.LOWERLIFT;
     break; 
     case LOWERLIFT:
     //TODO: FINISH STATATE
     boolean retracted=false;
     boolean rotated=false;
    if(m_rotate.getAbsolutePos()<Constants.Rotate.ABSHomePos){
      m_rotate.rotate(Constants.Rotate.rotateAutoInSpeed);
     }else{
            m_rotate.stopRotate();
            rotated=true;
     }
     if(m_elevator.getElevatorPos()<Constants.Lift.posLiftCubeTransfer){
      m_elevator.runWithOutLimit(Constants.Lift.liftAutoRetract2Speed);
     }else{     
          m_elevator.stop();
          retracted=true;;
     }
    if(rotated && retracted){
      m_state=STATE.PUSHCUBE;
    }
    break;
     

   
     case PUSHCUBE:
   //  if(m_starteCubePushTime=0){
      
    // }
     break; 
     case CLOSECLAW:
     m_pnuematics.setClaw(false);
     m_state=STATE.END;
     break;
     case ABORT:
     case END:
        m_elevator.stop();
        m_rotate.stopRotate();
        returnValue=true;

    }
    
    return returnValue;
  }
}
