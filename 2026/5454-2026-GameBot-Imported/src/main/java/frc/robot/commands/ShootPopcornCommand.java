package frc.robot.commands;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.HoodConstants;
import frc.robot.Constants.PassConstants.PassTargets;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.TurretSubsystemPots;
import frc.robot.subsystems.shooter.TurretUtil.TargetType;
import frc.robot.subsystems.shooter.PassLookUpTable.ShootingParameters;
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

  private CommandSwerveDrivetrain m_swerve;
  private NewShooterSubsystem m_shooter;
  private HopperSubsystem m_hopper;
  private IntakeSubsystem m_intake;
  private boolean m_isPass;
  private TurretSubsystemPots m_turret;
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

  public ShootPopcornCommand(CommandSwerveDrivetrain swerve, NewShooterSubsystem shooter, HopperSubsystem hopper, IntakeSubsystem intake, boolean isPass) {
    m_swerve=swerve;
    m_hopper=hopper;
    m_shooter=shooter;
    m_intake=intake;
    m_state=shooterStates.SPINUP;
    m_isPass=isPass;
    addRequirements(m_hopper);
    addRequirements(m_shooter);
    addRequirements(m_intake);
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
  System.out.println("Stopping Shooter");
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
  double distance=TurretUtil.getDistance(m_swerve.getPose2d(), TurretUtil.getNearestPassTargetType(m_swerve.getPose2d()));

    ShootingParameters shotParams = m_PassLookUpTable.getParameters(distance);
    
    targetspeed=shotParams.shooterSpeed;
    hoodPos=shotParams.hoodAngle;

  if(Math.abs(hoodPos-m_lastHoodPos)<Constants.HoodConstants.hoodDeadband) {
    hoodPos=m_lastHoodPos;
  }else {
    //update last position we moved to only if we are moving the hood Pos
    m_lastHoodPos=hoodPos;
  }

  
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
            // Capture the turret angle right before we start shooting
       //     m_heldTurretAngle = m_turret.getCurrentAngle();
            m_state=shooterStates.SHOOT;
        } 
        break;
    case SHOOT:
        m_shooter.runKicker(Constants.ShooterConstants.KickerSpeed);

        m_shooter.runNewShooter(targetspeed,
                            Constants.ShooterConstants.KickerSpeed);
       
        m_shooter.poormanHoldHoodPos(hoodPos, .06, 0.04);
        // Hold the turret at the captured angle to prevent drift during shooting
        // held pending testing
        //m_turret.holdTurretAtAngle(m_heldTurretAngle);
        m_hopper.agitate(Constants.HopperConstants.agitateSpeed);
        m_intake.runIntake(Constants.IntakeConstants.highSpeed);
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

