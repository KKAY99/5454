package frc.robot.commands;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.NoteFlipConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.NoteFlipperSubsystem;

public class AmpScoreHopeCommand extends Command {
    private ShooterSubsystem m_shooter;
    private NoteFlipperSubsystem m_flip;

   private enum STATE{SETANGLE,WAITFORANGLE,SHOOT,FEED,SHOOTANDFEED,END}
   private STATE m_state;

   private double m_angle;
   private double angleGap;
   private double m_currentTime;
   private double kTimeToRun=NoteFlipConstants.timeToRunAmpScore;

   public AmpScoreHopeCommand(ShooterSubsystem shooter,NoteFlipperSubsystem flip,double angle){
    m_shooter=shooter;
    m_flip=flip;
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
    m_flip.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    boolean returnValue=false;

    switch(m_state){
    case SETANGLE:    
        m_shooter.setAngle(m_angle);
        m_state=STATE.WAITFORANGLE;
    break;
    case WAITFORANGLE:
        angleGap=Math.abs(m_shooter.getRelativePosition())
        -Math.abs(m_angle);
        if(angleGap<Constants.ShooterConstants.kAngleDeadband&&angleGap>-Constants.ShooterConstants.kAngleDeadband){
            m_shooter.stopRotate();
            m_currentTime=Timer.getFPGATimestamp();
            m_state=STATE.SHOOTANDFEED;
        }
        break;
    case SHOOTANDFEED:
        m_shooter.RunShootingMotors(ShooterConstants.ampScoreSpeed,ShooterConstants.ampScoreSpeed,true);
        m_flip.run(NoteFlipConstants.noteFlipSpeed);

        if(m_currentTime>kTimeToRun+Timer.getFPGATimestamp()){
            m_shooter.stopShooter();
            m_currentTime=Timer.getFPGATimestamp();
            m_state=STATE.FEED;
        }
    break;
    case FEED:
        if(m_currentTime>kTimeToRun+Timer.getFPGATimestamp()){
            m_flip.stop();
            m_state=STATE.END;
        }
    break;
    case END:
     returnValue=true;
    break;
    }
    return returnValue;
  }
}
