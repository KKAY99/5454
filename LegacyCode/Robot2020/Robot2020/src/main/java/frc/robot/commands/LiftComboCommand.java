package frc.robot.commands;

import frc.robot.subsystems.DownLiftSubSystem;
import frc.robot.subsystems.UpLiftSubSystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * An example command that uses an example subsystem.
 */
public class LiftComboCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final DownLiftSubSystem m_downLift;
  private final UpLiftSubSystem m_upLift;
  private double m_downSpeed;
  private double m_upSpeed;
  private double m_downDuration;
  private double m_upDuration;
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public LiftComboCommand(DownLiftSubSystem downLift,UpLiftSubSystem upLift,double downDuration,double downSpeed,double upDuration,double upSpeed) {
    m_downLift = downLift;
    m_upLift=upLift;
    m_downSpeed=downSpeed;
    m_upSpeed=upSpeed;
    m_upDuration=upDuration;
    m_downDuration=downDuration;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_upLift);
    addRequirements(m_downLift);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println("Executing ComoboLift");
    //load cable
    m_downLift.setSpeed(m_downSpeed);
    Timer.delay(1);
    m_upLift.setSpeed(m_upSpeed);
    Timer.delay(m_upDuration);
    m_upLift.setSpeed(0);
    m_downLift.setSpeed(m_downSpeed);
    Timer.delay(m_downDuration);
    m_downLift.setSpeed(0);
    
  }

 // Called once the command ends or is interrupted.
 @Override
 public void end(final boolean interrupted) {
   m_upLift.setSpeed(0);
   m_downLift.setSpeed(0);
 }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
