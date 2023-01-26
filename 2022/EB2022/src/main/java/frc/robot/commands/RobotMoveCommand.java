package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
public class RobotMoveCommand extends CommandBase{
private DriveSubsystem m_drive;
private double m_leftSpeed;
private double m_rightSpeed;
    public RobotMoveCommand (DriveSubsystem driveSystem,double leftSpeed,double rightSpeed)
    {
        m_drive=driveSystem;
        m_leftSpeed=leftSpeed;
        m_rightSpeed=rightSpeed;
        addRequirements(m_drive);
    }
       // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  
  }
@Override
  public void execute() {
      m_drive.moveTank(m_leftSpeed, m_rightSpeed);
}
@Override
  public void end(final boolean interrupted) {
    m_drive.moveTank(0,0);
  }
  // Returns true when the command should end.
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
      return false;
    }
}
