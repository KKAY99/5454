package frc.robot.commands;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.shooter.NewShooterSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;

/** An example command that uses an example subsystem. */
public class ShootManualCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})  

  private NewShooterSubsystem m_shooter;
  private HopperSubsystem m_hopper;
  private enum shooterStates{
    SPINUP,WAIT,SHOOT,END
  } 
  private shooterStates m_state;
  private double stateStartTime;
  private final double kSpinUpTime=1;
  public ShootManualCommand(NewShooterSubsystem shooter,HopperSubsystem hopper) {
    m_hopper=hopper;
    m_shooter=shooter;
    m_state=shooterStates.SPINUP;
    addRequirements(m_hopper);
    addRequirements(m_shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_state=shooterStates.SPINUP;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  System.out.println("Stopping Shooter");
 
    m_shooter.stopNewShooter();
    m_hopper.stopAgitate();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

  boolean returnValue=false;

    
  
  System.out.println("Shooting - State:" + m_state);
    switch(m_state){
    case SPINUP:
        stateStartTime=Timer.getFPGATimestamp();
        m_shooter.runNewShooter(Constants.ShooterConstants.shootSpeed,
                            Constants.ShooterConstants.KickerSpeed);
        m_state=shooterStates.WAIT;
    break;
    case WAIT:
        double currentTime = Timer.getFPGATimestamp();
        if(currentTime>=stateStartTime+kSpinUpTime){
          m_state=shooterStates.SHOOT;
        }
      break;
    case SHOOT:
        m_hopper.agitate(Constants.HopperConstants.agitateSpeed);
        if(m_hopper.getNoFuel()) {
          m_state=shooterStates.END;
        }
    break;
    case END:
        returnValue=true;
    break;
  }
    return returnValue;

      
  }
}

