package frc.robot.commands;

import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class SwitchDriveModeCommand extends CommandBase {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final DriveSubsystem m_drive;
    private boolean m_driveReverse=false;
    
     
    public SwitchDriveModeCommand(DriveSubsystem driveSubsystem) {
      m_drive=driveSubsystem;
      // Use addRequirements() here to declare subsystem dependencies.
      addRequirements(m_drive);
    }
      // Called when the command is initially scheduled.
    @Override
    public void initialize() {
  
    }
  
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
      if(m_driveReverse==true){
        m_drive.setReverseMode();
        m_driveReverse=false;
      }
      else{
        m_drive.setForwardMode();
        m_driveReverse=true;
      }
      
      
    }
  
    // Called once the command ends or is interrupted.
    @Override
    public void end(final boolean interrupted) {
      }
  
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
      return true;
    }
  }