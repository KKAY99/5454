package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.RotateArmSubsystem;
import frc.robot.Constants;
import edu.wpi.first.hal.simulation.ConstBufferCallback;
import edu.wpi.first.wpilibj.Timer;

/** An example command that uses an example subsystem. */
 public class zMoveRotatePIDCommand extends CommandBase {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  //private final PaddleSubsystem m_IntakeSubsystem;

  private double m_rotateTarget = 0.0;
  private final int m_rotatePos;
  private double ktargetTolerance=5;
  private final RotateArmSubsystem m_rotate;
 
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  
  public zMoveRotatePIDCommand(RotateArmSubsystem rotate,int rotatePos) {
    m_rotate = rotate;  
    m_rotatePos = rotatePos;
    switch(rotatePos){
      case Constants.ChargedUp.middleTape:
      m_rotateTarget = Constants.Rotate.posMiddleConeFullLiftStage1;
      break;

      case Constants.ChargedUp.apriltag:
      m_rotateTarget = Constants.Rotate.posCubeOutofLimelight;
      break;
    }
  
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_rotate.SetPosAndMove(m_rotateTarget);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("MoveRotatePID Ending Pos is " + m_rotate.getRotatePos());
    m_rotate.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //is not finished until elevator is within tolerance of target
    return (Math.abs(m_rotate.getRotatePos()-m_rotateTarget)<ktargetTolerance);
 }
 }