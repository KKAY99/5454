package frc.robot.commands;

import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class DriveStraightCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final DriveSubsystem m_subsystem;
  private double m_speed;
  private double m_duration;
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public DriveStraightCommand(DriveSubsystem subsystem,double speed, double duration) {
    m_subsystem = subsystem;
    m_speed=speed;
    m_duration=duration;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println("Driving Straight");
    m_subsystem.commandDriveStraight(m_speed,m_duration);   
  }

 // Called once the command ends or is interrupted.
 @Override
 public void end(final boolean interrupted) {
  m_subsystem.tankDrive(0,0);
 }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
