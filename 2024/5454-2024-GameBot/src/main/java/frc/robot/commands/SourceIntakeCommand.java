package frc.robot.commands;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.NoteFlipConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.NoteFlipperSubsystem;

public class SourceIntakeCommand extends Command {
    private ShooterSubsystem m_shooter;

   private enum STATE{SETANGLE,WAITFORANGLE,INTAKE}
   private STATE m_state;

   private double m_angle;
   private double angleGap;

   public SourceIntakeCommand(ShooterSubsystem shooter,double angle){
    m_shooter=shooter;
    m_angle=angle;

   }

  @Override
  public void initialize() {
    m_state=STATE.SETANGLE;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted){
    m_shooter.stopRotate();
    m_shooter.stopShooter();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    switch (m_state){
    case SETANGLE:    
        m_shooter.setAngle(m_angle);
        m_state=STATE.WAITFORANGLE;
    break;
    case WAITFORANGLE:
        angleGap=Math.abs(m_shooter.getRelativePosition())
        -Math.abs(m_angle);
        if(angleGap<Constants.ShooterConstants.kAngleDeadband&&angleGap>-Constants.ShooterConstants.kAngleDeadband){
            m_shooter.stopRotate();
            m_state=STATE.INTAKE;
        }
        break;
    case INTAKE:
        m_shooter.RunShootingMotors(ShooterConstants.shooterIntakeSpeed,ShooterConstants.shooterIntakeSpeed,true,ShooterConstants.intakeFeederSpeed);
    break;
    }
    return false;
  }
}
