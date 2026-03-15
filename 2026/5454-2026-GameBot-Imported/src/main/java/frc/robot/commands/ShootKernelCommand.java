package frc.robot.commands;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.Constants.HoodConstants;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.shooter.NewShooterSubsystem;
import frc.robot.subsystems.shooter.TurretUtil.TargetType;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.TurretSubsystemPots;
/** An example command that uses an example subsystem. */
public class ShootKernelCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})  

  private NewShooterSubsystem m_shooter;
  private HopperSubsystem m_hopper;
  private IntakeSubsystem m_intake;
  private TurretSubsystemPots m_turret;
  private boolean m_emptyHopper=false;
  private double m_timeLimit=0;
  private enum shooterStates{
    SPINUP,WAIT,SHOOT, SHOOTING, NOFUEL,EMPTYHOPPER,NOFUEL2NDCHECK,END
  } 
  private shooterStates m_state;
  private double stateStartTime;
  private double startShootTime;
  private TargetType m_target;
  private final double kSpinUpTime=1;
  public ShootKernelCommand(NewShooterSubsystem shooter,HopperSubsystem hopper, IntakeSubsystem intake,double timeLimit,boolean emptyHopper,TurretSubsystemPots turret,TargetType target) {
    m_hopper=hopper;
    m_shooter=shooter;
    m_intake=intake;
    m_turret=turret;
    m_timeLimit=timeLimit;
    m_emptyHopper=emptyHopper;
    m_state=shooterStates.SPINUP;
    addRequirements(m_hopper);
    addRequirements(m_shooter);
    addRequirements(m_intake);
    addRequirements(m_turret);
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
 
    m_shooter.stopNewShooter(true);
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
          m_state=shooterStates.SHOOTING;
        }         
    break;
    case SHOOTING:
        double targetAngle =45;
        if(m_turret.isOnTargetAngle(targetAngle)){
          m_state=shooterStates.NOFUEL;
        } else {
          m_turret.turretTrack(m_target);
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
        if(m_intake.intakeCurrentLimitCheck(Constants.IntakeConstants.ampInStop)){
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
        m_shooter.hoodHome();
        returnValue=true;
    break;
  }
    return returnValue;

      
  }
}

