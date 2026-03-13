package frc.robot.commands;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.Constants.PassConstants.PassTargets;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.TurretSubsystemPots;
import frc.robot.subsystems.shooter.TurretUtil.TargetType;
import frc.robot.subsystems.shooter.NewShooterSubsystem;

/** An example command that uses an example subsystem. */
public class PassCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})  

  private NewShooterSubsystem m_shooter;
  private HopperSubsystem m_hopper;
  private IntakeSubsystem m_intake;
  private TurretSubsystemPots m_turret;
  private double m_timeLimit=0;
  private enum shooterStates{
    INTAKE,SPINUP,WAIT,SHOOT,NOFUEL,SHOOTING,END
  } 
  private shooterStates m_state;
  private double stateStartTime;
  private double startShootTime;
  private final double kSpinUpTime=1;
  private TargetType m_target;
  public PassCommand(NewShooterSubsystem shooter, HopperSubsystem hopper, IntakeSubsystem intake, TurretSubsystemPots turret, TargetType target, double timeLimit) {
    m_hopper=hopper;
    m_shooter=shooter;
    m_intake=intake;
    m_target=target;
    m_turret=turret;
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
          System.out.println("Pass Time Limit Reached... Ending Pass Command");
          returnValue=true;
        }
        return returnValue;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_state=shooterStates.INTAKE;
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
  

  System.out.println("Passing - State:" + m_state);
    switch(m_state){
    case INTAKE:
        m_intake.intakeonCommand();
        m_state=shooterStates.SPINUP;
    break;
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
        m_state=shooterStates.NOFUEL;         
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
        if(checkNoFuelorFuelTimeLimit()){
          m_state=shooterStates.END;
        }else{
          m_state=shooterStates.SHOOTING;
        }
    break;
    case END:
        CommandScheduler.getInstance().schedule(Commands.runOnce(()->m_shooter.hoodBack()));
        returnValue=true;
    break;
  }
    return returnValue;

      
  }
}

