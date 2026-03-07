package frc.robot.commands;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.shooter.NewShooterSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
/** An example command that uses an example subsystem. */
public class ShootManualCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})  

  private NewShooterSubsystem m_shooter;
  private HopperSubsystem m_hopper;
  private IntakeSubsystem m_intake;
  private boolean m_emptyHopper=false;
  private double m_timeLimit=0;
  private enum shooterStates{
    SPINUP,WAIT,SHOOT,NOFUEL,EMPTYHOPPER,NOFUEL2NDCHECK,END
  } 
  private shooterStates m_state;
  private double stateStartTime;
  private double startShootTime;
  private final double kSpinUpTime=1;
  public ShootManualCommand(NewShooterSubsystem shooter,HopperSubsystem hopper, IntakeSubsystem intake,double timeLimit,boolean emptyHopper) {
    m_hopper=hopper;
    m_shooter=shooter;
    m_intake=intake;
    m_emptyHopper=emptyHopper;
    m_state=shooterStates.SPINUP;
    addRequirements(m_hopper);
    addRequirements(m_shooter);
    addRequirements(m_intake);
  }

  private boolean checkNoFuelorFuelTimeLimit(){
    boolean returnValue=false;
    double currentTime;
        if(m_hopper.getNoFuel()) {
          System.out.println("No Fuel Detected...");
          returnValue=true;
        }
        //check time limit if the value is greater than zero
        //acts a failsafe if FuelSensor is not working
        currentTime = Timer.getFPGATimestamp();
        if(m_timeLimit>0 && (currentTime>=startShootTime+m_timeLimit)){
          System.out.println("Shoot Time Limit Reached... Ending Shoot Command");
          returnValue=true;
        }
        return returnValue;
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
    m_intake.intakeoffCommand();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
  double currentTime;
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
        currentTime = Timer.getFPGATimestamp();
        if(currentTime>=stateStartTime+kSpinUpTime){
          startShootTime=Timer.getFPGATimestamp();
          m_state=shooterStates.SHOOT;
        }
      break;
    case SHOOT:
        m_hopper.agitate(Constants.HopperConstants.agitateSpeed);
        m_intake.intakeonCommand();
        if(checkNoFuelorFuelTimeLimit()){
          m_state=shooterStates.NOFUEL;
        }         
    break;
    case NOFUEL:
        if(m_emptyHopper){
          m_state=shooterStates.EMPTYHOPPER;          
         } else{
          m_state=shooterStates.END;          
         }
      break; 
    case EMPTYHOPPER:
        if(m_intake.intakeCurrentLimitCheck()){
          m_state=shooterStates.NOFUEL2NDCHECK;
        }else {
          m_intake.inFold(Constants.IntakeConstants.foldSpeedAutoMode);  
          }
      break;
    case NOFUEL2NDCHECK:
        if(checkNoFuelorFuelTimeLimit()){
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

