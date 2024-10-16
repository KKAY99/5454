
package frc.robot.commands;
import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;
public class ManualShootCommand extends Command {
  private ShooterSubsystem m_shooter;
  private double m_power;
  public ManualShootCommand(ShooterSubsystem shooter,double power) {
    m_shooter = shooter;
    m_power = power;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_shooter.runShooterMotors(m_power);
   // m_shooter.runFeeder(Constants.Shooter.ShooterFeederSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooter.stopShooterMotors();
    //m_shooter.stopFeeder();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
