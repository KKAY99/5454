package frc.robot.commands;

import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class DriveRobotCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final DriveSubsystem m_subsystem;
  private double m_direction;
  private double m_speed;
  private double m_duration;
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public DriveRobotCommand(DriveSubsystem subsystem,double direction, double speed, double duration) {
    m_subsystem = subsystem;
    m_speed=speed;
    m_duration=duration;
    m_direction=direction;
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
    if (m_direction == 0){
     m_subsystem.commandDriveStraight(m_speed,m_duration);   
    }else if (m_direction==180){
      
      m_subsystem.commandDriveStraight(-m_speed, m_duration);
    }else if (m_direction>0){
    }else if (m_direction<180){

    }

    
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
