package frc.robot.commands;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.commands.ClimbAutoAlign;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;

/** An example command that uses an example subsystem. */
public class ClimbCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})  
  private final ClimbSubsystem m_climb;
  private final CommandSwerveDrivetrain m_swerve;
  private enum climbStates{
    EXTEND, LINEUP, CLIMB, END};
  private climbStates m_state;

  public ClimbCommand(ClimbSubsystem climbSubsystem, CommandSwerveDrivetrain swerveSubsystem) {
    m_climb = climbSubsystem;
    m_swerve = swerveSubsystem;
     //couldn't figure out how to do this without creating auto align here in the constructor so
     Command lineUp = new ClimbAutoAlign(true, m_swerve);
    m_state = climbStates.EXTEND;
    addRequirements(m_climb);
    addRequirements(m_swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_state = climbStates.EXTEND;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("Climb Ending");
    m_climb.stopClimb();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    boolean returnValue=false;


    System.out.println("Climb State: "+m_state);
    switch(m_state){
      case EXTEND:
        m_climb.climbUpCommand();
        if(m_climb.isClimbUpLimit()){
          m_climb.stopClimb();
          m_state=climbStates.LINEUP;
        }
        break;
      case LINEUP:
        Commands.run(lineUp()).schedule();
        if(m_climb.getLineupLimit()){
          m_climb.stopClimb();
          m_state=climbStates.CLIMB;
        }
        break;
      case CLIMB:
        m_climb.climbUp();
        if(m_climb.getClimbLimit()){
          m_climb.stopClimb();
          m_state=climbStates.END;
        }
        break;
      case END:
        returnValue=true;
        break;
    }
    return false;
  }
}

