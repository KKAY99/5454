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
/** An example command that uses an example subsystem. */
public class TurretTrackCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})  

  private TurretSubsystemPots m_turret;
  private CommandSwerveDrivetrain m_drive;
  
  private TurretStates m_turretState;
  private PoseCalculator m_PoseCalc = new PoseCalculator();
  public TurretTrackCommand(TurretSubsystemPots turret,CommandSwerveDrivetrain drive,TurretStates state) {
    m_turret=turret;    
    m_drive=drive;
    m_turretState=state;
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

  private Pose2d getHubPose(){
    //default to Blue Hub for target
    Pose2d hubTarget=Constants.LocationConstants.blueHub;
    Optional<Alliance> ally = DriverStation.getAlliance();
   if (ally.isPresent()) {
        if (ally.get() == Alliance.Red) {
            hubTarget=Constants.LocationConstants.redHub;
        }
      }
    return hubTarget;
  }
  private void trackHub(){
    Pose2d currentPose = m_drive.getPose2d();
    Pose2d aimAtPose = getHubPose();
    double bearing = m_PoseCalc.getBearingAngle( currentPose,aimAtPose);
    ShotSolution solution = TurretUtil.computeShotSolution(currentPose, TargetType.HUB);
    System.out.println("Solution " + solution.turretAngleDegrees);
    double currentAngle = m_turret.getCurrentAngle();
    double angleVariance = Math.abs(solution.turretAngleDegrees - currentAngle);
    if(angleVariance>TurretConstants.trackerDeadBand){
        String outputString="Current Pose " + String.format("%.2f",currentPose.getX()) + "," + String.format("%.2f",currentPose.getY())
                            + "Aim at Pose:" + String.format("%.2f",aimAtPose.getX()) + "," + String.format("%.2f",aimAtPose.getY()); 
        SmartDashboard.putString("Pose Info", outputString);                                                            
        SmartDashboard.putString("Turret Move", "Moving Turret " + currentAngle +
                                " to " + bearing);
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

