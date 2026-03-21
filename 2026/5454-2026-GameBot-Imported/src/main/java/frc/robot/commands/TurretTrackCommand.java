package frc.robot.commands;
import java.util.Optional;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.TurretSubsystemPots;
import frc.robot.utilities.PoseCalculator;
import frc.robot.Constants.TurretStates;
import frc.robot.Constants;
import frc.robot.Constants.TurretConstants;
import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.subsystems.shooter.TurretUtil;
import frc.robot.subsystems.shooter.TurretUtil.ShotSolution;
import frc.robot.subsystems.shooter.TurretUtil.TargetType;
import frc.robot.utilities.Limelight;
/** An example command that uses an example subsystem. */
public class TurretTrackCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})  

  private TurretSubsystemPots m_turret;
  private CommandSwerveDrivetrain m_drive;
  private Limelight m_limelight;
  private TurretStates m_turretState;
  private PoseCalculator m_PoseCalc = new PoseCalculator();
  private final double kLimelightDeadband=0.5;

  public TurretTrackCommand(TurretSubsystemPots turret,CommandSwerveDrivetrain drive,TurretStates state,Limelight limelight) {
    m_turret=turret;    
    m_drive=drive;
    m_turretState=state;
    m_limelight=limelight;
    addRequirements(m_turret);
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
   
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_turret.stopTurret();
  }

  
  private void trackHub(){
    if(!m_limelight.isFilteredTargetAvailable()){
      
      double llx=m_limelight.getX();
      
      if(Math.abs(llx)<kLimelightDeadband){
        m_turretState=TurretStates.END;
        m_turret.stopTurret();
      } else {
        if (llx>0){
          if(m_turret.getTurretPOTS()>TurretConstants.TurretLeftLimitPOTS){
            m_turret.moveTurret(TurretConstants.kTrackingSpeed);
            } else {
               m_turret.stopTurret();
            }
        } else{
           if(m_turret.getTurretPOTS()>Constants.TurretConstants.TurretRightLimitPOTS){
            m_turret.moveTurret(TurretConstants.kTrackingSpeed);
          } else {
            m_turret.stopTurret();
          }
       }
      }
    } 
  }
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    boolean returnValue=false;
    switch(m_turretState){
      case MANUAL:
        m_turret.stopTurret();
        returnValue=true;
        break;
        
      case TRACK:
        trackHub();
        break;
      case FIXEDLEFT:
        break;
      case FIXEDRIGHT:
        break;
    } 
    return returnValue;
  }
}

