package frc.robot.commands;
import frc.robot.Constants;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.LiftSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;



/** An example command that uses an example subsystem. */
public class zMoveArmExtend extends CommandBase {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  
  private final LiftSubsystem m_liftSubsystem;
  private double m_posInitLift;
  private double m_posFullLiftStage1;
  private double m_posFullLiftStage2;
  private double m_angleStage1;
  private double m_angleStage2;
  private static enum STATE
  {
                  INITLIFT,ROTATE,EXTENDANDROTATE,EXTENDLIFT,ABORT,END
  }
 private STATE m_state=STATE.INITLIFT;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public zMoveArmExtend(LiftSubsystem lift,Constants.ChargedUP.TargetLevels targetLevel) {
    m_liftSubsystem=lift;
  // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_liftSubsystem);
    switch(targetLevel){
      case HIGH:
      m_posFullLiftStage1=Constants.Lift.posHighFullLiftStage1;
      m_posFullLiftStage2=Constants.Lift.posHighFullLiftStage2;
      m_angleStage1=Constants.Rotate.angleHighConeStage1;
      m_angleStage2=Constants.Rotate.angleHighConeStage2;
      break;
      case MiDDLE:
        m_posFullLiftStage1=Constants.Lift.posMiddleFullLiftStage1;
        m_posFullLiftStage2=Constants.Lift.posMiddleFullLiftStage2;
        m_angleStage1=Constants.Rotate.angleMiddleConeStage1;
        m_angleStage2=Constants.Rotate.angleMiddleConeStage2;
      break;
      case LOW:
        m_posFullLiftStage1=Constants.Lift.posLowFullLiftStage1;
        m_posFullLiftStage2=Constants.Lift.posLowFullLiftStage2;
        m_angleStage1=Constants.Rotate.angleLowConeStage1;
        m_angleStage2=Constants.Rotate.angleLowConeStage2;
      break;
    }
    m_posInitLift=Constants.Lift.posInitLift;
     
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_state=STATE.INITLIFT;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  System.out.println("Move Arm Extend " + m_state.toString());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
      m_liftSubsystem.stopElevator();
      m_liftSubsystem.stopRotate();
      }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    
    boolean returnValue=false;
    System.out.println(m_state);
    switch(m_state){
      case INITLIFT:
          
          // Encoder is negative as it lifts up
          if(m_liftSubsystem.getElevatorPos()>m_posInitLift){
            m_liftSubsystem.runElevator(Constants.Lift.liftAutoExtendStage1Speed);
          } else{
            m_liftSubsystem.stopElevator();
            m_state=STATE.EXTENDANDROTATE;
          // m_state=STATE.ABORT;
          }
          returnValue=false;
          break;
      case ROTATE:
         if(m_liftSubsystem.getRotatePos()<m_angleStage2){
          m_liftSubsystem.rotate(Constants.Rotate.rotateAutoOutStage2Speed);
         } else{
          m_liftSubsystem.stopRotate();
          m_state=STATE.EXTENDLIFT;
         }
         returnValue=false;
          break;
      case EXTENDLIFT:
       // Encoder is negative as it lifts up
       if(m_liftSubsystem.getElevatorPos()>m_posFullLiftStage2){
        m_liftSubsystem.runElevator(Constants.Lift.liftAutoExtendStage2Speed);
      } else{
        m_liftSubsystem.stopElevator();
        m_state=STATE.END;
      }
     
        returnValue=false;
        break;
      case EXTENDANDROTATE:
       boolean extended=false;
       boolean rotated=false;
      if(m_liftSubsystem.getRotatePos()<m_angleStage1){
        m_liftSubsystem.rotate(Constants.Rotate.rotateAutoOutStage1Speed);
       } else{
          if(m_liftSubsystem.getRotatePos()<m_angleStage2){
            m_liftSubsystem.rotate(Constants.Rotate.rotateAutoOutStage2Speed);  
            } else{
              m_liftSubsystem.stopRotate();
              rotated=true;
             }
      }
       if(m_liftSubsystem.getElevatorPos()>m_posFullLiftStage1){
        m_liftSubsystem.runElevator(Constants.Lift.liftAutoExtendStage1Speed);
      } else{
          if(m_liftSubsystem.getElevatorPos()>m_posFullLiftStage2){
            m_liftSubsystem.runElevator(Constants.Lift.liftAutoExtendStage2Speed);
          }else{
            
            m_liftSubsystem.stopElevator();
            extended=true;;
          }
        }
      if(rotated && extended){
        m_state=STATE.END;
      }
      break;
      case ABORT:
      case END:
        m_liftSubsystem.stopElevator();
        m_liftSubsystem.stopRotate();
        returnValue=true;

    }
    
    return returnValue;
  }
}
