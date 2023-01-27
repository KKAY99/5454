package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
public class AutoMoveCommand extends CommandBase{
private DriveSubsystem m_drive;
private double m_speed;
private double m_time;
    public AutoMoveCommand (DriveSubsystem driveSystem,double speed, double time)
    {
        m_drive=driveSystem;
        m_speed=speed;
        m_time=time;
        addRequirements(m_drive);
    }
@Override
  public void execute() {
      m_drive.commandDriveStraight(m_speed, m_time);
}
@Override
  public void end(final boolean interrupted) {
    m_drive.commandDriveStraight(0,0);
  }
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
