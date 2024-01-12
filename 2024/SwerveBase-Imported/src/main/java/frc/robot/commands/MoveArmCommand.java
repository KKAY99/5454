package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.RotateArmSubsystem;

/** An example command that uses an example subsystem. */
public class MoveArmCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"}) 
  
  private RotateArmSubsystem m_rotateArm;
  private double m_speed;

  public MoveArmCommand(RotateArmSubsystem rotateArm,double speed) {
    m_rotateArm=rotateArm;
    m_speed=speed;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize(){}

  // Called every time the scheduler runs while the command is scheduled.

  @Override
  public void end(boolean interrupted){
    m_rotateArm.stopRotate();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    m_rotateArm.rotate(m_speed);
    return false;
  }
}

