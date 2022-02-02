package frc.robot.commands;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeLiftSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.Constants;
import frc.robot.Constants.TeleOpAutomationConstants;
import frc.robot.Constants.IntakeConstants;
public class TeleopMoveShootCommand extends CommandBase{
private DriveSubsystem m_drive;
private IntakeLiftSubsystem m_intakeLift;
private IntakeSubsystem m_intake;
private boolean m_autoShoot;
    public TeleopMoveShootCommand (DriveSubsystem driveSystem,IntakeSubsystem intake, IntakeLiftSubsystem intakeLift, boolean autoShoot)
    {
        m_drive=driveSystem;
        m_intake=intake;
        m_intakeLift=intakeLift;
        m_autoShoot=autoShoot;
        addRequirements(m_drive);
    }
@Override
  public void execute() {
    m_drive.commandDrive(TeleOpAutomationConstants.moveLeftSpeed,TeleOpAutomationConstants.moveRightSpeed,TeleOpAutomationConstants.moveGoalTime);
    m_intakeLift.setSpeed(Constants.IntakeLiftSpeeds.intakeLiftUpSpeedSlow);
    Timer.delay(.5);     
    //move half speed for last part of driving forward
    m_drive.commandDrive(TeleOpAutomationConstants.moveLeftSpeed/2,TeleOpAutomationConstants.moveRightSpeed/2,TeleOpAutomationConstants.moveLastGoalTime);
    if (m_autoShoot){
      m_intakeLift.setSpeed(Constants.IntakeLiftSpeeds.intakeLiftUpSpeedSlow);
      Timer.delay(.5);     
      //move half speed for last part of driving forward
      m_drive.commandDrive(TeleOpAutomationConstants.moveLeftSpeed/2,TeleOpAutomationConstants.moveRightSpeed/2,TeleOpAutomationConstants.moveLastGoalTime);
      m_intakeLift.setSpeed(0);
      m_intake.setSpeed(IntakeConstants.intakeSpeed);
      Timer.delay(2);   
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
