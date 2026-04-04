package frc.robot.commands;
import javax.lang.model.util.ElementScanner14;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.Constants.HoodConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.TurretSubsystemPots;
import frc.robot.subsystems.shooter.HubLookUpTable;
import frc.robot.subsystems.shooter.NewShooterSubsystem;
import frc.robot.subsystems.shooter.TurretUtil;
import frc.robot.subsystems.shooter.HubLookUpTable.ShootingParameters;
import frc.robot.subsystems.shooter.TurretUtil.ShotSolution;
import frc.robot.subsystems.shooter.TurretUtil.TargetType;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.utilities.Limelight;
/** An example command that uses an example subsystem. */
public class ShotOnTheMoveCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})  

  private NewShooterSubsystem m_shooter;
  private HopperSubsystem m_hopper;
  private IntakeSubsystem m_intake;
  private TurretSubsystemPots m_turret;
  private Limelight m_limelight;
  private HubLookUpTable m_HubLookUpTable = new HubLookUpTable();
  private boolean m_emptyHopper=false;
  private double m_lastHoodPos=0;
  private double m_lastDistance=60;//24; // default distance to use so will take mid shot if limelight is not responding
  private double m_timeLimit=0;
  private double overrideDistance=0;
  private boolean overrideDistanceFlag=false;
  //private double m_heldTurretAngle=0; // Angle to hold the turret at during shooting
  private enum shooterStates{
    SPINUP,WAIT,SHOOT,NOFUEL,EMPTYHOPPER,SHOOTMORE,NOFUEL2NDCHECK,END
  } 
  private shooterStates m_state;
  private double stateStartTime;
  private double startShootTime;
  private final double kSpinUpTime=1;
  private final double khoodSpeed=Constants.HoodConstants.hoodSpeed;
  private final double khoodDeadband = Constants.HoodConstants.hoodDeadband;
  private double fuelcheckStartTime;
  private final double kfuelcheckWait=2;
  private int m_flipCount=0;
  private int m_flipCountLimit=0;
  private final int kflipCountMax=6;//35;
  private final int kHopperPullLimit=14;
  private CommandSwerveDrivetrain m_swerve;
  private int m_HopperPulls=0;
  //private boolean NoLimeLightMode=0;
  private double m_flipSpeed=0;
  private double kTurretPosDeadband=0.1;
  public ShotOnTheMoveCommand(CommandSwerveDrivetrain swerve,NewShooterSubsystem shooter, HopperSubsystem hopper, IntakeSubsystem intake, 
                          TurretSubsystemPots turret,Limelight limelight, double timeLimit, boolean emptyHopper) {
    m_hopper=hopper;
    m_shooter=shooter;
    m_intake=intake;
    m_swerve=swerve;
    m_turret=turret;
    m_limelight=limelight;
    overrideDistanceFlag=false;
    
    m_emptyHopper=emptyHopper;
    m_state=shooterStates.SPINUP;
    addRequirements(m_hopper);
    addRequirements(m_shooter);
    addRequirements(m_intake);
  }

  public ShotOnTheMoveCommand(TurretSubsystemPots turret,CommandSwerveDrivetrain swerve,NewShooterSubsystem shooter, HopperSubsystem hopper, IntakeSubsystem intake, 
                            double timeLimit) {
    m_hopper=hopper;
    m_shooter=shooter;
    m_intake=intake;
    m_swerve=swerve;
    m_turret=turret;
    m_state=shooterStates.SPINUP;
    addRequirements(m_hopper);
    addRequirements(m_shooter);
    addRequirements(m_intake);
    //DO NOT REQUIRE TURRET OR DRIVE
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
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  //System.out.println("Stopping Shooter");
    m_shooter.hoodMoveToZero();
    m_intake.stopFold();
    m_intake.SetIntakeOutMode();
    m_shooter.stopNewShooter(true);
    m_hopper.stopAgitate();
    m_intake.stopIntake();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
  double currentTime;
  boolean returnValue=false;

double velX = m_swerve.getChassisSpeeds().vxMetersPerSecond;
double velY = m_swerve.getChassisSpeeds().vyMetersPerSecond;
ShotSolution targetShot = TurretUtil.computeLeadShotSolution(m_swerve.getPose2d(),velX,velY,TurretUtil.TargetType.HUB); 
double targetspeed=targetShot.shooterSpeedRPS;
double hoodPos=targetShot.trajectoryAngleDegrees;
double turretSourceAngle=targetShot.turretAngleDegrees;

//REFERENCE ONLY - DO NOT USE TARGETING
 double StaticTurretAngleTarget=TurretUtil.get5454TurretAngle(m_swerve.getPose2d(),TurretUtil.TargetType.HUB);
  //always adjust the angle
   double angle=TurretUtil.get5454TurretAngleFromAngle(turretSourceAngle);
         
  //System.out.println("Shooting On the Move - Speed "  + targetspeed  + 
  //                   "Target Angle:" + angle + " Static Angle:" + StaticTurretAngleTarget +" - State:" + m_state);
  SmartDashboard.putNumber("Turret Util Target Angle",angle);
  double targetPos=m_turret.getTargetMotorPosition(angle);
  SmartDashboard.putNumber("Turret Util Target Pos",targetPos); 
  m_turret.moveMotor(targetPos); 
  
switch(m_state){
    case SPINUP:
         m_shooter.poormanHoldHoodPos(hoodPos, khoodSpeed,khoodDeadband); 
         //m_shooter.holdHoodPosMotionMagic(hoodPos);
        if(m_shooter.checkHoodPos(hoodPos, khoodSpeed,khoodDeadband)){
         stateStartTime=Timer.getFPGATimestamp();
          
         m_shooter.runNewShooter(targetspeed,
                            0);
          m_state=shooterStates.WAIT;
        }
    break;
    case WAIT:
       m_shooter.poormanHoldHoodPos(hoodPos, .06, 0.04); 
       //m_shooter.holdHoodPosMotionMagic(hoodPos);
        if(m_shooter.atTargetSpeed(targetspeed) && checkTurretPos(targetPos)){
            m_state=shooterStates.SHOOT;
        } 
        break;
    case SHOOT:
        if(checkTurretPos(targetPos)==false){
          //stop kicker until the angle is alligned
            m_shooter.runNewShooter(targetspeed,
                      0);
            m_state=shooterStates.WAIT;
        } else {          
           m_shooter.runKicker(Constants.ShooterConstants.KickerSpeed);

          m_shooter.runNewShooter(targetspeed,
                            Constants.ShooterConstants.KickerSpeed);
       
          m_shooter.poormanHoldHoodPos(hoodPos, .06, 0.04);
          //m_shooter.holdHoodPosMotionMagic(hoodPos);
        
          m_hopper.agitate(Constants.HopperConstants.agitateSpeed);
         m_intake.runIntake(Constants.IntakeConstants.highSpeed);
        //STAY IN SHOOT
        
        } 
       break;
    case NOFUEL:
        fuelcheckStartTime=Timer.getFPGATimestamp();
        if(m_emptyHopper){
          m_flipCount=0; 
          m_flipCountLimit=0;
          m_flipSpeed=0;
          m_HopperPulls=0;
          m_state=shooterStates.EMPTYHOPPER;    
          //m_intake.inFold(Constants.IntakeConstants.foldSpeedAutoMode);      
         } else{
          m_state=shooterStates.END;          
         }
      break; 
    case EMPTYHOPPER:
        m_shooter.poormanHoldHoodPos(hoodPos, .06, 0.04);
        //m_shooter.holdHoodPosMotionMagic(hoodPos);
    
    /*  //System.out.println("Flip Count"+ m_flipCount);
        m_flipCount=m_flipCount+1;
        if (m_flipCount==m_flipCountLimit){
          //make it twice as fast
          m_intake.inFold(Constants.IntakeConstants.foldSpeedAutoMode * 3 *  m_flipSpeed);
          m_flipCount = 0;
          m_flipSpeed=m_flipSpeed*-1; // FLIP SIGN TO REVERSE
          if(m_flipCountLimit<kflipCountMax){
            m_flipCountLimit=m_flipCountLimit+10;
          }
        }
        /*if(m_intake.isAtInLimit() || m_intake.intakeCurrentLimitCheck(Constants.IntakeConstants.ampInStop)){
          m_state=shooterStates.NOFUEL2NDCHECK;
          m_intake.stopFold();
        }*/
        //if flip count (times through the loop) is greeater than limit than move to next hopper
        if(m_flipCount>m_flipCountLimit){
          m_flipCount=0;
          m_HopperPulls=m_HopperPulls+1;
        }
        
        switch(m_HopperPulls){
          case 0:
            m_flipSpeed=-0.8; // coming in speed
            m_flipCountLimit=3;
          break;
          case 1:
            m_flipSpeed=0.8;  //Out speed
            m_flipCountLimit=2;
          break;
          case 2:
            m_flipSpeed=-0.8;
            m_flipCountLimit=8;
          break;
          case 3:
            m_flipSpeed=0.8; 
            m_flipCountLimit=6;
          break;
          case 4:
            m_flipSpeed=-0.8;
            m_flipCountLimit=12;
          break;
          default:
           if (m_HopperPulls % 2 != 0) {  //EDIT if we change hopper pull limit
              m_flipSpeed=0.8; //Out speed which we should start with first in the default case since we end with an inward pull
              m_flipCountLimit=4;
            } else {
              m_flipSpeed=-0.8; // coming in speed
              m_flipCountLimit=6; // We pull in further then we push out incase we had jamed and never pulled in enough to start
            }
          break;
        }


        m_flipCount=m_flipCount+1;
        
        //System.out.println("Flip Count:" + m_flipCount + " Hopper Pulls: "+ m_HopperPulls + " Speed:"+ m_flipSpeed);
   
        m_intake.inFold(m_flipSpeed);
        
        if(m_intake.isinNoFlyZone()){
          m_intake.stopIntake();
        } else {
          m_intake.runIntake(Constants.IntakeConstants.highSpeed);
        }
        
        if(m_HopperPulls>kHopperPullLimit){
          //System.out.println("Stop Folding");
          m_intake.stopFold();
          m_state=shooterStates.SHOOTMORE;
        }
      break;
    case SHOOTMORE:
            m_shooter.poormanHoldHoodPos(hoodPos, .06, 0.04);
            //m_shooter.holdHoodPosMotionMagic(hoodPos);
        //STAY IN THE LOOP FOREVER UNTIL USER STOPS
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

  private boolean checkTurretPos(double targetPosition){
    boolean returnValue=false;
    double actual=m_turret.getCurrentPosition();
    if(Math.abs(targetPosition-actual)<kTurretPosDeadband){
      returnValue=true;
    }
    return returnValue;
  }
  
}

