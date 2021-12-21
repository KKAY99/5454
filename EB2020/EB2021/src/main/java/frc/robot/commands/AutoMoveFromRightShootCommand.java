package frc.robot.commands;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeLiftSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.Constants;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.IntakeConstants;
public class AutoMoveFromRightShootCommand extends CommandBase{
private DriveSubsystem m_drive;
private IntakeLiftSubsystem m_intakeLift;
private IntakeSubsystem m_intake;
private boolean m_autoShoot;
    public AutoMoveFromRightShootCommand (DriveSubsystem driveSystem,IntakeSubsystem intake, IntakeLiftSubsystem intakeLift, boolean autoShoot)
    {
        m_drive=driveSystem;
        m_intake=intake;
        m_intakeLift=intakeLift;
        m_autoShoot=autoShoot;
        addRequirements(m_drive);
    }
@Override
  public void execute() {
    m_intakeLift.setSpeed(-0.2);
    Timer.delay(.5);
    m_intakeLift.setSpeed(0);
    //move forward one robot
    m_drive.commandDrive(AutoConstants.moveLeftSpeed,AutoConstants.moveRightSpeed,AutoConstants.moveRightStep1);
    //turn left
    m_drive.commandDrive(-AutoConstants.moveTurnSpeed,-AutoConstants.moveTurnSpeed,AutoConstants.moveRightStep2);
    //move forward
    m_drive.commandDrive(AutoConstants.moveLeftSpeed,AutoConstants.moveRightSpeed,AutoConstants.moveRightStep3);
    //turn right to straigten out
    m_drive.commandDrive(AutoConstants.moveTurnSpeed,AutoConstants.moveTurnSpeed,AutoConstants.moveRightStep4);
    //runn into wall
    m_drive.commandDrive(AutoConstants.moveLeftSpeed,AutoConstants.moveRightSpeed,AutoConstants.moveRightStep5);
    
    if (m_autoShoot){
      m_intakeLift.setSpeed(Constants.IntakeLiftSpeeds.intakeLiftUpSpeedSlow);
      Timer.delay(1);     
      m_drive.commandDrive(AutoConstants.moveLeftSpeed,AutoConstants.moveRightSpeed,AutoConstants.moveLastGoalTime);
      m_intakeLift.setSpeed(0);
      m_intake.setSpeed(IntakeConstants.intakeSpeed);
      Timer.delay(3);
      m_intake.setSpeed(0);
      m_drive.commandDriveStraight(AutoConstants.moveSpeed, AutoConstants.moveBackup);
    }
}
@Override
  public void end(final boolean interrupted) {
    m_drive.commandDriveStraight(0,0);
  }
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
