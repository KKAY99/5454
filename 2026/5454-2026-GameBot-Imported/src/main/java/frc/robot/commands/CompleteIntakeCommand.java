package frc.robot.commands;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.shooter.NewShooterSubsystem;

/** An example command that uses an example subsystem. */
public class CompleteIntakeCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})  

  private IntakeSubsystem m_intake;
  private HopperSubsystem m_hopper;
  private NewShooterSubsystem m_shooter;
  private enum intakeStates{
    INTAKE,END
  } 
  private intakeStates m_state;
  private double stateStartTime;
  private final double kSpinUpTime=1;
  public CompleteIntakeCommand(IntakeSubsystem intake,HopperSubsystem hopper,NewShooterSubsystem shooter) {
    m_hopper=hopper;
    m_intake=intake;
    m_state=intakeStates.INTAKE;
    addRequirements(m_hopper);
    addRequirements(m_intake);
    }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_state=intakeStates.INTAKE;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  System.out.println("Stopping Intake and Hopper");
 
    m_intake.stopIntake();
    m_hopper.stopAgitate();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

  boolean returnValue=false;

    
  
    switch(m_state){
    case INTAKE:
        m_hopper.agitate(Constants.HopperConstants.agitateSpeed);
        m_intake.runIntake(Constants.IntakeConstants.highSpeed);     
    break;
    
    case END:
        returnValue=true;
    break;
  }
    return returnValue;

      
  }
}