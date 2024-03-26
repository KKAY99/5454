package frc.robot.commands;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.NoteFlipConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.NoteFlipperSubsystem;

public class AmpScoreHopeCommand extends Command {
    private ShooterSubsystem m_shooter;
    private IntakeSubsystem m_intake;
    private NoteFlipperSubsystem m_flip;

   private enum STATE{STARTNOTEMOTOR,SETANGLE,WAITFORANGLE,STARTFEEDER,SHOOTANDFEED,MOVEUPANGLE,WAITFORMOVEUPANGLE,END}
   private STATE m_state;

   private double m_angle;
   private double angleGap;
   private double m_startTime;
   private double kGetGoing=-0.15;
   private double kTimeToRun=NoteFlipConstants.timeToRunAmpScore;

   public AmpScoreHopeCommand(ShooterSubsystem shooter,IntakeSubsystem intake,NoteFlipperSubsystem flip,double angle){
    m_shooter=shooter;
    m_intake=intake;
    m_flip=flip;
    m_angle=angle;
   }

  @Override
  public void initialize() {
    m_state=STATE.STARTNOTEMOTOR;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted){
    m_shooter.stopRotate();
    m_intake.stopIntake();
    m_shooter.stopShooter();
    m_flip.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    boolean returnValue=false;

    switch(m_state){
    case STARTNOTEMOTOR:
         m_flip.run(NoteFlipConstants.startNoteFlipSpeed);
         m_state=STATE.SETANGLE;
         break;
    case SETANGLE:    
        m_shooter.setAngle(m_angle);       
        m_state=STATE.WAITFORANGLE;
        break;
    case WAITFORANGLE:
        angleGap=(m_shooter.getRelativePosition())
        -m_angle;
        if(angleGap<Constants.ShooterConstants.kAngleDeadband&&angleGap>-Constants.ShooterConstants.kAngleDeadband){
            m_shooter.stopRotate();
            m_startTime=Timer.getFPGATimestamp();
            m_state=STATE.STARTFEEDER;
        }
        break;
    case STARTFEEDER:
        m_flip.run(NoteFlipConstants.noteFlipSpeed); //now that it is moving slow it
        m_state=STATE.SHOOTANDFEED;    
        break;
    case SHOOTANDFEED:
        m_shooter.RunShootingMotors(ShooterConstants.ampScoreSpeed,ShooterConstants.ampScoreSpeed,true);
        m_intake.runIntake(IntakeConstants.intakeSpeed);
        if(m_startTime+kTimeToRun<Timer.getFPGATimestamp()){
            m_shooter.stopShooter();
            m_intake.stopIntake();
            m_state=STATE.MOVEUPANGLE;
        }
    break;
    case MOVEUPANGLE:    
        m_shooter.setAngle(0);
        m_state=STATE.WAITFORMOVEUPANGLE;
    break;
    case WAITFORMOVEUPANGLE:
        angleGap=Math.abs(m_shooter.getRelativePosition())
        -Math.abs(0);
        if(angleGap<Constants.ShooterConstants.kAngleDeadband&&angleGap>-Constants.ShooterConstants.kAngleDeadband){
            m_shooter.stopRotate();
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
