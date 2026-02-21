package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.TurretSubsystemPots;

/** An example command that uses an example subsystem. */
public class MoveTurretCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})  

  private TurretSubsystemPots m_turret;
  private double m_speed;

  public MoveTurretCommand(TurretSubsystemPots turret,double speed) {
    m_turret=turret;    
    m_speed=speed;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_turret.moveTurret(m_speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_turret.stopTurret();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_turret.atLimit(m_speed);
  }
}

