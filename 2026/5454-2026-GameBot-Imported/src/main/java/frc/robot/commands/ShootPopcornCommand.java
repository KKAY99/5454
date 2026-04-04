package frc.robot.commands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.Constants.HoodConstants;
import frc.robot.Constants.InputControllers;
import frc.robot.Constants.PassConstants.PassTargets;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.TurretSubsystemPots;
import frc.robot.subsystems.shooter.TurretUtil.TargetType;
import frc.robot.subsystems.shooter.PassLookUpTable.ShootingParameters;
import frc.robot.subsystems.shooter.HubLookUpTable;
import frc.robot.subsystems.shooter.NewShooterSubsystem;
import frc.robot.subsystems.shooter.PassLookUpTable;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.shooter.TurretUtil;

/*I want this to:
  2. aim at that target ; be spinning up
  3. shoot at the target ; run allat stuff
 */


/** An example command that uses an example subsystem. */
public class ShootPopcornCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})  

  private PassLookUpTable m_PassLookUpTable = new PassLookUpTable();
  private HubLookUpTable m_HubLookupTable = new HubLookUpTable();
  private NewShooterSubsystem m_shooter;
  private HopperSubsystem m_hopper;
  private IntakeSubsystem m_intake;
  private boolean m_isPass;
  
  private double m_lastDistance; // default distance
  private double m_lastHoodPos=0; // default hood pos
  private enum shooterStates{
    SPINUP,WAIT,SHOOT,PASSING,END
  } 
  private double stateStartTime;
  private shooterStates m_state;
  private final double kSpinUpTime=1;

  private final double khoodSpeed=Constants.HoodConstants.hoodSpeed;
  private final double khoodDeadband = Constants.HoodConstants.hoodDeadband;

  public ShootPopcornCommand(NewShooterSubsystem shooter, HopperSubsystem hopper, IntakeSubsystem intake, boolean isPass) {
    m_hopper=hopper;
    m_shooter=shooter;
    m_intake=intake;
    m_state=shooterStates.SPINUP;
    m_isPass=isPass;
    addRequirements(m_hopper);
    addRequirements(m_shooter);
    addRequirements(m_intake);
    //DRIVE NOT added by design
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (m_isPass) {
      m_lastDistance=Constants.ShooterConstants.kDefaultPassDistance;
    } else {
      m_lastDistance=Constants.ShooterConstants.kDefaultShootingDistance;
    }
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
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
  boolean returnValue=false;
  //Lookup Shot Values
  double targetspeed=0;
  double hoodPos=0;  
  double distance;
  Pose2d turretPose;
  turretPose=m_shooter.getRobotPose();
  //System.out.println(turretPose.getX());
  if(m_isPass){
      PassLookUpTable.ShootingParameters passParams;
       distance=TurretUtil.getDistance(m_shooter.getRobotPose(), TurretUtil.getNearestPassTargetType(m_shooter.getRobotPose()));
      passParams = m_PassLookUpTable.getParameters(distance);
      targetspeed=passParams.shooterSpeed;
      hoodPos=passParams.hoodAngle;

  } else { 
    HubLookUpTable.ShootingParameters shotParams;
    distance=TurretUtil.getDistance(m_shooter.getRobotPose(),TurretUtil.TargetType.HUB);
    shotParams=m_HubLookupTable.getParameters(distance);
    targetspeed=shotParams.shooterSpeed;
    hoodPos=shotParams.hoodPosition;
  }
    
  SmartDashboard.putNumber("Shot Distance ",distance);
SmartDashboard.putNumber("Shot Speed",targetspeed);
SmartDashboard.putNumber("Shot Hood",hoodPos);
  if(Math.abs(hoodPos-m_lastHoodPos)<Constants.HoodConstants.hoodDeadband) {
    hoodPos=m_lastHoodPos;
  }else {
    //update last position we moved to only if we are moving the hood Pos
    m_lastHoodPos=hoodPos;
  }

  
    switch(m_state){
    case SPINUP:
        //start intake
        m_intake.runIntake(Constants.IntakeConstants.highSpeed); 
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
        /* if(m_shooter.atTargetSpeed(targetspeed)==false){
    
          //stop kicker until the speed is up to speed 
          //allows for moving robot to pause shooting for new distance
            m_shooter.runNewShooter(targetspeed,
                      0);
            m_state=shooterStates.WAIT;
        } else { */          
          //Ready to Shoot
          m_shooter.runKicker(Constants.ShooterConstants.KickerSpeed);

          m_shooter.runNewShooter(targetspeed,
                              Constants.ShooterConstants.KickerSpeed);
        
          m_shooter.poormanHoldHoodPos(hoodPos, .06, 0.04);
          m_hopper.agitate(Constants.HopperConstants.agitateSpeed);
          m_intake.runIntake(Constants.IntakeConstants.highSpeed);
        //}
       break;
    case PASSING:
            m_shooter.poormanHoldHoodPos(hoodPos, .06, 0.04);
        //STAY IN THE LOOP FOREVER UNTIL USER STOPS
     break;
    case END:
        m_shooter.hoodHome();
        returnValue=true;
    break;
  }
    return returnValue;

      
  }
}

