package frc.robot.commands;


import frc.robot.subsystems.DriveSubsystem;
import frc.robot.Constants;
import frc.robot.Limelight;
import frc.robot.Constants.LimeLightValues;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class MovetoDistanceCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final DriveSubsystem m_drive;
  private final double m_targetDistance;
  private boolean m_complete;
  /**
    * @param targetSpeed The speed we are setting in execute
   */
  public MovetoDistanceCommand(DriveSubsystem driveSystem,double targetDistance) {
    m_drive=driveSystem;
    m_targetDistance = targetDistance;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveSystem);
  }
    // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_complete=false;

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
   double currentDistance;
   double currentAngle;
   //currentAngle=
 
  
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(final boolean interrupted) {
}
   

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_complete;
  }
}

