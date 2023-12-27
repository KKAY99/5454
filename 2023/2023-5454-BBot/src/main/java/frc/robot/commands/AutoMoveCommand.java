package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.*;
import frc.robot.Constants;
/** An example command that uses an example subsystem. */
public class AutoMoveCommand extends CommandBase {
  private final SwerveSubsystem m_drive;
  private final double m_direction;
  private final double m_distance;
  private final double m_rcw;
  private boolean m_useNavX=false;
  private boolean m_isFinished=false;
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})    
  public AutoMoveCommand(SwerveSubsystem subsystem,double direction,double distance) {
    m_drive=subsystem;
    m_direction=direction;
    m_distance=distance;
    m_rcw=0; // default value - not passed in
    m_useNavX=false;
    addRequirements(subsystem);
  }
  public AutoMoveCommand(SwerveSubsystem subsystem,double direction,double rcw,double distance) {
    m_drive=subsystem;
    m_direction=direction;m_distance=distance;
    m_rcw=rcw;
    m_useNavX=false;
    addRequirements(subsystem);
  }

  public AutoMoveCommand(SwerveSubsystem subsystem,double direction) {
    m_drive=subsystem;
    m_direction=0;
    m_distance=0;
    m_rcw=0;
 
    m_useNavX=true;    
    addRequirements(subsystem);
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_isFinished=false;
    System.out.println("Try Auto Drive " + m_distance);
    //if(m_useNavX){
      //m_drive.spin(m_direction,Constants.AutoModes.MoveSpeed);
   // }else {
       m_drive.move(m_direction,Constants.AutoModes.MoveSpeed,m_distance,true);
   // }
       m_isFinished=true;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_isFinished;
  }
}

