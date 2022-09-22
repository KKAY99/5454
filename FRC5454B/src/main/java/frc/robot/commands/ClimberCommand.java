package frc.robot.commands;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimbSubsystem;

public class ClimberCommand extends CommandBase {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private ClimbSubsystem m_subsystem;    
    private double m_speed;
    /**
      * @param targetSpeed The speed we are setting in execute
     */
    public ClimberCommand(ClimbSubsystem climber,double speed) {
      m_subsystem=climber;
      m_speed=speed;
      // Use addRequirements() here to declare subsystem dependencies.
      addRequirements(m_subsystem);
    }
      // Called when the command is initially scheduled.
    @Override
    public void initialize() {
  
    }
  
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        m_subsystem.runClimb(m_speed);
    }
  
    // Called once the command ends or is interrupted.
    @Override
    public void end(final boolean interrupted) {
        m_subsystem.stopClimb();
        
    }
  
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
      return false;
    }    
}

