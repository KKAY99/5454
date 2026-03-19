package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.Constants.HoodConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.shooter.HubLookUpTable.ShootingParameters;
import frc.robot.subsystems.shooter.HubLookUpTable;
import frc.robot.subsystems.shooter.NewShooterSubsystem;
import frc.robot.subsystems.shooter.TurretUtil;
import frc.robot.subsystems.shooter.TurretUtil.ShotSolution;
import frc.robot.subsystems.shooter.TurretUtil.TargetType;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import edu.wpi.first.math.geometry.Pose2d;
/** An example command that uses an example subsystem. */
import frc.robot.subsystems.TurretSubsystemPots;
import frc.robot.utilities.Limelight;

public class ShootPopcornCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})  

  private CommandSwerveDrivetrain m_swerve;
  private NewShooterSubsystem m_shooter;
  private HopperSubsystem m_hopper;
  private IntakeSubsystem m_intake;
  private TurretSubsystemPots m_turret;
  private HubLookUpTable m_HubLookUpTable = new HubLookUpTable();
  private double m_lastHoodPos=0;
  private double m_lastDistance=24; // default distance to use so will take up close shot if limelight is blocked/broken/hosed 
  private double m_timeLimit=0;
  private Limelight m_limelight3;  
  private enum shooterStates{
    INTAKE, SPINUP,WAIT,AGITATE,SHOOTING,END
  } 
  private shooterStates m_state;
  private double stateStartTime;
  private double startShootTime;
  private final double khoodSpeed=Constants.HoodConstants.hoodSpeed;
  private final double khoodDeadband = Constants.HoodConstants.hoodDeadband;
  private TargetType m_target;
  private final double kSpinUpTime=1;
  
  public ShootPopcornCommand(NewShooterSubsystem shooter, HopperSubsystem hopper, IntakeSubsystem intake, 
                              TurretSubsystemPots turret, CommandSwerveDrivetrain swerve, Limelight limelight,
                              TargetType target) {
    m_hopper=hopper;
    m_shooter=shooter;
    m_intake=intake;
    m_turret=turret;
    m_target=target;
    m_swerve=swerve;
    m_limelight3=limelight;
 
    m_state=shooterStates.INTAKE;
    addRequirements(m_hopper);
    addRequirements(m_shooter);
    addRequirements(m_intake);
    addRequirements(m_turret);
    //DO NOT ADDD SWERVE AS A REQUIREMENT AS WE WANT TO BE ABLE TO DRIVE WHILE IN SHOOTING MODE.
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
  
 
  System.out.println("Popcorn Shooting - State:" + m_state);
    switch(m_state){
    case INTAKE:
        m_intake.intakeonCommand();
        m_state=shooterStates.SPINUP;
    break;
    case SPINUP:
        stateStartTime=Timer.getFPGATimestamp();
        //run shooter no kicker
        m_shooter.runNewShooter(Constants.ShooterConstants.shootSpeed,
                            0);
        m_state=shooterStates.WAIT;
    break;
    case WAIT:
        currentTime = Timer.getFPGATimestamp();
        if(currentTime>=stateStartTime+kSpinUpTime){
          startShootTime=Timer.getFPGATimestamp();
          m_state=shooterStates.AGITATE;
        }
      break;
    case AGITATE:
        m_hopper.agitate(Constants.HopperConstants.agitateSpeed);
        m_intake.intakeonCommand();
         // Statys in SHootPopcornMode until interrupted             
        m_state=shooterStates.SHOOTING;
        break;
    case SHOOTING:
        // Track the hub using Limelight AprilTag vision
          if(m_limelight3.isHubVisible()){
            // Get the bearing angle to the hub from Limelight
            double hubBearing = m_limelight3.getHubBearingAngle();
            
            // Command the turret to track the hub
            m_turret.trackTurret(hubBearing);

            // Lookup shooting parameters
              double distance=m_limelight3.getDistance();
              //if distance is zero than use last disance 
              if(distance==0){
                  distance=m_lastDistance;
              }else {
                    m_lastDistance=distance;
              }
              //update params based on distance
              ShootingParameters shotParams = m_HubLookUpTable.getParameters(distance);

              targetspeed=shotParams.shooterSpeed;
              hoodPos=shotParams.hoodPosition;
              if(Math.abs(hoodPos-m_lastHoodPos)<Constants.HoodConstants.hoodDeadband) {
                hoodPos=m_lastHoodPos;
              }else {
                //update last position we moved to only if we are moving the hood Pos
                m_lastHoodPos=hoodPos;
              }
            
            // Bring hood to position & hold
              m_shooter.poormanHoldHoodPos(hoodPos, khoodSpeed,khoodDeadband); 

            // Now shoot
              m_shooter.runKicker(Constants.ShooterConstants.KickerSpeed);
              m_shooter.runNewShooter(targetspeed,Constants.ShooterConstants.KickerSpeed);
              m_shooter.poormanHoldHoodPos(hoodPos, .06, 0.04); 
              m_hopper.agitate(Constants.HopperConstants.agitateSpeed);
              m_intake.runIntake(Constants.IntakeConstants.highSpeed);


          } else {
            // If hub is not visible, stop turret movement and keep shooter idling
              m_turret.stopTurret();
              m_shooter.runShooterVelocity(Constants.ShooterConstants.shootSpeed);
              m_hopper.agitate(Constants.HopperConstants.agitateSpeed);
              m_intake.runIntake(Constants.IntakeConstants.highSpeed);
          }
        // Stay in Tracking/Shooting state until the command is interrupted
        break;
    case END:
        m_shooter.hoodHome();
        returnValue=true;
    break;
  }
    return returnValue;

      
  }
}

