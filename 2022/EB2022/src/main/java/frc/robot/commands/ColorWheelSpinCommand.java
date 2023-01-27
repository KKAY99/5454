package frc.robot.commands;

import frc.robot.subsystems.ColorWheelSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ColorWheelSpinCommand extends CommandBase {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private ColorWheelSubsystem m_subsystem;
    private double m_targetSpeed;
    
    /**
      * @param targetSpeed The speed we are setting in execute
     */
    public ColorWheelSpinCommand(ColorWheelSubsystem ColorSubsystem,double targetSpeed) {
      m_subsystem=ColorSubsystem;
      m_targetSpeed = targetSpeed;
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
      m_subsystem.setSpeed(m_targetSpeed);
  
    }
  
    // Called once the command ends or is interrupted.
    @Override
    public void end(final boolean interrupted) {
      m_subsystem.setSpeed(0);
    }
  
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
      return false;
    }
  }