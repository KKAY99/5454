package frc.robot.commands;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.Constants.HoodConstants;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.shooter.HubLookUpTable;
import frc.robot.subsystems.shooter.NewShooterSubsystem;
import frc.robot.subsystems.shooter.HubLookUpTable.ShootingParameters;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.utilities.Limelight;
/** An example command that uses an example subsystem. */
public class ShotLookupCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})  

  private NewShooterSubsystem m_shooter;
  private HopperSubsystem m_hopper;
  private IntakeSubsystem m_intake;
  private Limelight m_limelight;
  private HubLookUpTable m_HubLookUpTable = new HubLookUpTable();
  private boolean m_emptyHopper=false;
  private double m_lastHoodPos=0;
  private double m_lastDistance=24; // default distance to use so will take up close shot if limelight is blocked/broken/hosed 
  private double m_timeLimit=0;
  private enum shooterStates{
    SPINUP,WAIT,SHOOT,NOFUEL,EMPTYHOPPER,NOFUEL2NDCHECK,END
  } 
  private shooterStates m_state;
  private double stateStartTime;
  private double startShootTime;
  private final double kSpinUpTime=1;
  private final double khoodSpeed=Constants.HoodConstants.hoodSpeed;
  private final double khoodDeadband = Constants.HoodConstants.hoodDeadband;
  private double fuelcheckStartTime;
  private final double kfuelcheckWait=2;
  
  public ShotLookupCommand(NewShooterSubsystem shooter,HopperSubsystem hopper, IntakeSubsystem intake,Limelight limelight, double timeLimit,boolean emptyHopper) {
    m_hopper=hopper;
    m_shooter=shooter;
    m_intake=intake;
    m_limelight=limelight;
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
    m_shooter.hoodMoveToZero();
  
    m_shooter.stopNewShooter(true);
    m_hopper.stopAgitate();
    m_intake.stopIntake();;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
  double currentTime;
  boolean returnValue=false;
  
  //Lookup Shot Values
  double targetspeed=0;
  double hoodPos=0;  
  double distance=m_limelight.getDistance();
  //if distance is zero than use last disance 
  if(distance==0){
       distance=m_lastDistance;
  }else {
        m_lastDistance=distance;
  }
  //limelight is unable to see the camera 
  ShootingParameters shotParams = m_HubLookUpTable.getParameters(distance);

  targetspeed=shotParams.shooterSpeed;
  hoodPos=shotParams.hoodPosition;
  if(Math.abs(hoodPos-m_lastHoodPos)<Constants.HoodConstants.hoodDeadband) {
    hoodPos=m_lastHoodPos;
  }else {
    //update last position we moved to only if we are moving the hood Pos
    m_lastHoodPos=hoodPos;
  }
  
  System.out.println("Shooting Lookup :" + distance + " Speed:" + targetspeed  + " - State:" + m_state);
    switch(m_state){
    case SPINUP:
         m_shooter.poormanHoldHoodPos(hoodPos, khoodSpeed,khoodDeadband); 
        if(m_shooter.checkHoodPos(hoodPos, khoodSpeed,khoodDeadband)){
         stateStartTime=Timer.getFPGATimestamp();
          
         m_shooter.runNewShooter(targetspeed,
                            0);
          m_state=shooterStates.WAIT;
        }
    break;
    case WAIT:
       m_shooter.poormanHoldHoodPos(hoodPos, .06, 0.04); 
        if(m_shooter.atTargetSpeed(targetspeed)){
            m_state=shooterStates.SHOOT;
        } 
        break;
    case SHOOT:
        m_shooter.runKicker(Constants.ShooterConstants.KickerSpeed);

        m_shooter.runNewShooter(targetspeed,
                            Constants.ShooterConstants.KickerSpeed);
       
        m_shooter.poormanHoldHoodPos(hoodPos, .06, 0.04); 
        m_hopper.agitate(Constants.HopperConstants.agitateSpeed);
        m_intake.runIntake(Constants.IntakeConstants.highSpeed);
       
        if(checkNoFuelorFuelTimeLimit()){
          m_state=shooterStates.NOFUEL;
        }         
       break;
    case NOFUEL:
        fuelcheckStartTime=Timer.getFPGATimestamp();
        if(m_emptyHopper){
          m_state=shooterStates.EMPTYHOPPER;          
         } else{
          m_state=shooterStates.END;          
         }
      break; 
    case EMPTYHOPPER:
        if(m_intake.isAtInLimit() | 
                 m_intake.intakeCurrentLimitCheck(Constants.IntakeConstants.ampInStop)){
          m_state=shooterStates.NOFUEL2NDCHECK;
          m_intake.stopFold();
        }else {
           if(m_intake.isinNoFlyZone()){
              m_intake.stopIntake();
          }
          m_intake.inFold(Constants.IntakeConstants.foldSpeedAutoMode);  
          }
      break;
    case NOFUEL2NDCHECK:
        currentTime=Timer.getFPGATimestamp();
        if(currentTime>fuelcheckStartTime+kfuelcheckWait){
          if(checkNoFuelorFuelTimeLimit() ){
            m_state=shooterStates.END;
          }
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

