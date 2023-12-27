package frc.robot.commands;
import frc.robot.Constants;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.LiftSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;



/** An example command that uses an example subsystem. */
public class zMoveArmRetract extends CommandBase {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  
  private final LiftSubsystem m_liftSubsystem;
  private static enum STATE
  {
                  RETRACTANDROTATE,RETURNLIFT,ABORT,END
  }
 private STATE m_state=STATE.RETRACTANDROTATE;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public zMoveArmRetract(LiftSubsystem lift) {
    m_liftSubsystem=lift;
  // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_liftSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_state=STATE.RETRACTANDROTATE;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
   
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

      case RETRACTANDROTATE:
      boolean retracted=false;
      boolean rotated=false;
     if(m_liftSubsystem.getRotatePos()>Constants.Rotate.angleIntakePos){
       m_liftSubsystem.rotate(Constants.Rotate.rotateAutoInSpeed);
      } else{
        
             m_liftSubsystem.stopRotate();
             rotated=true;
     }
      if(m_liftSubsystem.getElevatorPos()<Constants.Lift.posInitLift){
       m_liftSubsystem.runElevator(Constants.Lift.liftAutoRetractSpeed);
     } else{     
           m_liftSubsystem.stopElevator();
           retracted=true;;
         }
     if(rotated && retracted){
       m_state=STATE.RETURNLIFT;
     }
     break;
     case RETURNLIFT:
    
          // Encoder is negative as it lifts up
          if(m_liftSubsystem.getElevatorPos()<Constants.Lift.posHome){
            m_liftSubsystem.runElevator(Constants.Lift.liftAutoRetractHomeSpeed);
          } else{
            m_liftSubsystem.stopElevator();
            m_state=STATE.END;
          }
          returnValue=false;
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
