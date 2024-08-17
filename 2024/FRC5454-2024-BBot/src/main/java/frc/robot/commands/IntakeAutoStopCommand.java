

package frc.robot.commands;
import frc.robot.Constants;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants.SteerFeedbackType;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Shooter;
import frc.robot.classes.BreakBeam;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
public class IntakeAutoStopCommand extends Command {
  private IntakeSubsystem m_intake;
  private ShooterSubsystem m_shooter;
  private BreakBeam m_breakBeam;

  private double m_power;

  private enum States{
    INTAKE,WAITFORBREAKBEAM,END
  }

  private States m_currentState=States.INTAKE;

  public IntakeAutoStopCommand(IntakeSubsystem intake, ShooterSubsystem shooter, BreakBeam breakBeam, double power) {
    m_intake = intake;
    m_power = power;
    m_shooter = shooter;
    m_breakBeam=breakBeam;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute(){
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intake.stop();
    m_shooter.stopFeeder();
    m_currentState=States.INTAKE;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    boolean returnValue=false;

    switch(m_currentState){
      case INTAKE:
        m_intake.run(m_power);
        m_shooter.runFeeder(Constants.Shooter.ShooterFeederSpeed);

        m_currentState=States.WAITFORBREAKBEAM;
      break;
      case WAITFORBREAKBEAM:
        //if(m_breakBeam.GetBreakBeam()){
          m_currentState=States.END;
        //}

        //System.out.println(m_breakBeam.GetBreakBeam());
      break;
      case END:
        returnValue=true;
      break;
    }
    return returnValue;
  }
}
