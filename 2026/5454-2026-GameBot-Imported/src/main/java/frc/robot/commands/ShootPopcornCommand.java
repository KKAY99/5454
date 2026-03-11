package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.shooter.NewShooterSubsystem;
import frc.robot.subsystems.shooter.TurretUtil;
import frc.robot.subsystems.shooter.TurretUtil.ShotSolution;
import frc.robot.subsystems.shooter.TurretUtil.TargetType;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import edu.wpi.first.math.geometry.Pose2d;
/** An example command that uses an example subsystem. */
import frc.robot.subsystems.TurretSubsystemPots;
public class ShootPopcornCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})  

  private NewShooterSubsystem m_shooter;
  private HopperSubsystem m_hopper;
  private IntakeSubsystem m_intake;
  private TurretSubsystemPots m_turret;
  private CommandSwerveDrivetrain m_swerve;  
  private enum shooterStates{
    INTAKE, SPINUP,WAIT,SHOOT,SHOOTING,END
  } 
  private shooterStates m_state;
  private double stateStartTime;
  private double startShootTime;
  private TargetType m_target;
  private final double kSpinUpTime=1;
  public ShootPopcornCommand(NewShooterSubsystem shooter,HopperSubsystem hopper, IntakeSubsystem intake,TurretSubsystemPots turret, CommandSwerveDrivetrain swerve,TargetType target) {
    m_hopper=hopper;
    m_shooter=shooter;
    m_intake=intake;
    m_turret=turret;
    m_target=target;
    m_swerve=swerve;
 
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
 
    m_shooter.stopNewShooter(true);
    m_hopper.stopAgitate();
    m_intake.intakeoffCommand();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
  double currentTime;
  boolean returnValue=false;

 
  System.out.println("Popcorn Shooting - State:" + m_state);
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
         // Statys in SHootPopcornMode until interrupted             
        m_state=shooterStates.SHOOTING;
        m_shooter.runShooterVelocity(Constants.ShooterConstants.IdleSpeed);
        break;
    case SHOOTING:
        ShotSolution solution = 
        TurretUtil.computeShotSolution(m_swerve.getPose2d(), m_target);
        if(solution.isValid){}
          if(m_turret.isOnTargetAngle(solution.turretAngleDegrees)){
            m_shooter.runNewShooter(solution.shooterSpeedRPS,
                            Constants.ShooterConstants.KickerSpeed);
            
          } else {
            m_turret.turretTrack(m_target);
          }
        break;
    case END:
        returnValue=true;
    break;
  }
    return returnValue;

      
  }
}

