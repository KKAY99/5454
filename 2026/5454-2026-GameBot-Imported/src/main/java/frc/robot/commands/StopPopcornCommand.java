package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.TurretSubsystemPots;
import frc.robot.subsystems.shooter.NewShooterSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
/** An example command that uses an example subsystem. */
public class StopPopcornCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})  

  private NewShooterSubsystem m_shooter;
  private HopperSubsystem m_hopper;
  private IntakeSubsystem m_intake;
  private TurretSubsystemPots m_turret;
  public StopPopcornCommand(NewShooterSubsystem shooter,HopperSubsystem hopper, IntakeSubsystem intake,TurretSubsystemPots turret) {
    m_hopper=hopper;
    m_shooter=shooter;
    m_intake=intake;
    m_turret=turret;
    addRequirements(m_hopper);
    addRequirements(m_shooter);
    addRequirements(m_intake);
    addRequirements(m_turret);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  //nothing to execute . command ends after one iteration as isFinished is always true.
  //code to stop mode is in end
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  System.out.println("Stopping Shooter");
 
    m_shooter.stopNewShooter();
    m_hopper.stopAgitate();
    m_intake.intakeoffCommand();
    m_turret.stopTurret();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
  return true;       
  }
}

