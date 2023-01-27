package frc.robot.commands;


import frc.robot.subsystems.ClimberSubsystem;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;

public class ClimberLimitCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ClimberSubsystem m_subsystem;
  private final double m_targetSpeed;
  private DigitalInput m_limitSwitch=new DigitalInput(Constants.LimitSwitches.ArmDown);
  private Counter m_limitCounter=new Counter(m_limitSwitch);
   
  /**
   * 
    * @param targetSpeed The speed we are setting in execute
   */
  public ClimberLimitCommand(ClimberSubsystem Climbersubsystem,final double targetSpeed) {
    m_subsystem=Climbersubsystem;
    m_targetSpeed = targetSpeed;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_subsystem);
  }
    // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_limitCounter.reset();;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {  
    if(m_limitSwitch.get()==false) {
      System.out.println("Setting climber speed = "+ m_targetSpeed);
      m_subsystem.setSpeed(m_targetSpeed);
    } else {
        System.out.println("Limit Switch Hitch" );
         m_subsystem.setSpeed(0);

    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(final boolean interrupted) {
    m_subsystem.setSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //if counter has incremented then move up
    return (m_limitCounter.get()>0);
  }
}

