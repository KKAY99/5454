

package frc.robot.commands;
import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Shooter;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
public class OuttakeCommand extends Command {
  private IntakeSubsystem m_intake;
  private ShooterSubsystem m_shooter;

  private double m_power;

  public OuttakeCommand(IntakeSubsystem intake, ShooterSubsystem shooter,double power) {
    m_intake = intake;
    m_power = power;
    m_shooter = shooter;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_intake.run(m_power);
    m_shooter.runFeeder(Constants.Shooter.ShooterFeederSpeed);
    m_shooter.runShooterMotors(Constants.Shooter.ShooterOutTakeSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intake.stop();
    m_shooter.stopFeeder();
    m_shooter.stopShooterMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
