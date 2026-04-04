package frc.robot.commands;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.shooter.NewShooterSubsystem;
import frc.robot.subsystems.shooter.TurretUtil;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.utilities.Limelight;
/** An example command that uses an example subsystem. */
public class ShootMappingCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})  

  private NewShooterSubsystem m_shooter;
  private HopperSubsystem m_hopper;
  private IntakeSubsystem m_intake;
  private Limelight m_limelight;
  private boolean m_emptyHopper=false;
  private double m_timeLimit=0;
  private enum shooterStates{
    SPINUP,WAIT,SHOOT,END
  } 
  private shooterStates m_state;
  private double m_speed=0;
  private double m_idleSpeed=0;
  private double m_hoodPos=0;
  private double stateStartTime;
  private double startShootTime;
  CommandSwerveDrivetrain m_swerve;
  private final double kSpinUpTime=1;
  public ShootMappingCommand(CommandSwerveDrivetrain swerve,NewShooterSubsystem shooter,HopperSubsystem hopper, IntakeSubsystem intake,Limelight limelight, double timeLimit,boolean emptyHopper) {
    m_hopper=hopper;
    m_shooter=shooter;
    m_intake=intake;
    m_swerve=swerve;
    m_limelight=limelight;
    m_emptyHopper=emptyHopper;
    m_state=shooterStates.SPINUP;
    SmartDashboard.putNumber("Idle Speed",20);
    SmartDashboard.putNumber("Target Speed",40);
    SmartDashboard.putNumber("Hood Target Position",0);
    addRequirements(m_hopper);
    addRequirements(m_shooter);
    addRequirements(m_intake);
  }

  private boolean checkNoFuelorFuelTimeLimit(){
    boolean returnValue=false;
    double currentTime;
        if(m_hopper.getNoFuel()) {
          //System.out.println("No Fuel Detected...");
          returnValue=true;
        }
        //check time limit if the value is greater than zero
        //acts a failsafe if FuelSensor is not working
        currentTime = Timer.getFPGATimestamp();
        if(m_timeLimit>0 && (currentTime>=startShootTime+m_timeLimit)){
          //System.out.println("Shoot Time Limit Reached... Ending Shoot Command");
          returnValue=true;
        }
        return returnValue;
  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_state=shooterStates.SPINUP;
    getValues();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  //System.out.println("Stopping Shooter");
    //disabled idle mode for hood testing
    m_shooter.stopNewShooter(true);
    m_hopper.stopAgitate();
    m_intake.stopIntake();
    m_shooter.stopHood();
    
  }
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
  double currentTime;
  boolean returnValue=false;  
  double distance=TurretUtil.getDistance(m_swerve.getPose2d(), TurretUtil.TargetType.HUB);
  SmartDashboard.putNumber("Odom Distance",distance);
   
  //System.out.println("Shot Mapping - State:" + m_state  + " " + Timer.getFPGATimestamp());
    switch(m_state){
    case SPINUP:
        m_shooter.poormanHoldHoodPos(m_hoodPos, .06, 0.04); 
        if(m_shooter.checkHoodPos(m_hoodPos, .06, 0.04)){
         stateStartTime=Timer.getFPGATimestamp();
          m_shooter.runNewShooter(m_speed,
                            Constants.ShooterConstants.KickerSpeed);
          m_state=shooterStates.WAIT;
        }
        
        
    break;
    case WAIT:
        m_shooter.poormanHoldHoodPos(m_hoodPos, .06, 0.04); 
        if(m_shooter.atTargetSpeed(m_speed)){
            m_state=shooterStates.SHOOT;
        }
      break;
    case SHOOT:
        m_shooter.poormanHoldHoodPos(m_hoodPos, .06, 0.04); 
        m_hopper.agitate(Constants.HopperConstants.agitateSpeed);
        m_intake.runIntake(Constants.IntakeConstants.highSpeed);
        break;
    case END:
        m_shooter.hoodHome();
        returnValue=true;
    break;
  }
    return returnValue;

      
  }
  
    private void getValues() {
      double idleSpeed= SmartDashboard.getNumber("Idle Speed",20);
      double targetSpeed=SmartDashboard.getNumber("Target Speed",40);
      double hoodPos = SmartDashboard.getNumber("Hood Target Position",0);
      m_idleSpeed=idleSpeed;
      m_speed=targetSpeed;
      m_hoodPos=hoodPos;

  }

}

