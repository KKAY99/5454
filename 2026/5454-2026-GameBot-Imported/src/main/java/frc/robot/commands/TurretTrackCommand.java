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
  private TurretStates m_initState;
  private PoseCalculator m_PoseCalc = new PoseCalculator();
  private final double kLimelightDeadband=0.5;
  private final double kTurretFastSpeed=0.05454;
  private final double kTurretSearchSpeed=0.08454;
 
  private final double kTurretSlowSpeed=0.03;
  private boolean m_SearchRight=false;
  private boolean m_flipTrack=false;

  public TurretTrackCommand(TurretSubsystemPots turret,CommandSwerveDrivetrain drive,TurretStates state,Limelight limelight, Boolean flipTrack) {
    m_turret=turret;    
    m_drive=drive;
    m_turretState=state;
    m_initState=state;
    m_limelight=limelight;
    m_flipTrack=flipTrack;
    addRequirements(m_turret);
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //filter on front of hubs
    m_limelight.setLimelightIDFilter(10,26);
    m_turretState=m_initState;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
   
  }

  //Four states turret tracking can be
  // m_SearchRight     |      reversed
  //   True                     False  - Normal operation, run the first if statement then set m_SearchRight to false at end
  //   True                     True   - Reversed of the normal, run the second if statement and m_SearchRight to false at the end
  //   False                    True   - Reversed operation, run the first if statement then set m_SearchRight to true at the end
  //   False                    False  - Normal operation, run the second if statement then set m_SearchRight to true at the end

  private void turretSearch(boolean reversed) {
    if(m_limelight.isAnyTargetAvailable()){
      m_turretState=TurretStates.TRACK;
    }else {
      if((m_SearchRight && !reversed) || (!m_SearchRight && reversed)){
          if(m_turret.getTurretPOTS()<TurretConstants.TurretRightLimitPOTS){
            m_turret.moveTurret(kTurretSearchSpeed);
          } else {
            m_turret.stopTurret();
            m_SearchRight = reversed ? true : false;
          }
        } else {
            System.out.println("Move R");
          if(m_turret.getTurretPOTS()>Constants.TurretConstants.TurretLeftLimitPOTS){
            System.out.println("Moving");
            m_turret.moveTurret(-kTurretSearchSpeed);
          } else {
            System.out.println("Left Limit");
            m_turret.stopTurret();
            m_SearchRight = reversed ? false : true;
        }
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("Ending Turret Track");
    m_turret.stopTurret();
  }

  
  private void trackHub(boolean reversed){
    System.out.println("Tracking Turret XY");
 //   if(m_limelight.isFilteredTargetAvailable()){
    if(m_limelight.isAnyTargetAvailable() ){
      
      double llx=m_limelight.getX();
      System.out.println("Limelight X:"  + llx + " POTS:" + m_turret.getTurretPOTS());
      double gap=Math.abs(llx)-kLimelightDeadband;
      double turretSpeed;
      if(gap>1){
         turretSpeed=kTurretFastSpeed;
      }else {
        turretSpeed=kTurretSlowSpeed;
      }
      if(Math.abs(llx)<kLimelightDeadband){
        System.out.println("In Limelight Deadband");
        m_turretState=TurretStates.END;
        m_turret.stopTurret();
      } else {
        if (llx<0){
          if(m_turret.getTurretPOTS()<TurretConstants.TurretRightLimitPOTS){
            //Move Left
            m_turret.moveTurret(turretSpeed);
            } else {
               System.out.println("Right Limit");
               m_turret.stopTurret();
            }
        } else{
           //Move Right
           System.out.println("Move R");
           if(m_turret.getTurretPOTS()>Constants.TurretConstants.TurretLeftLimitPOTS){
            System.out.println("Moving");
            m_turret.moveTurret(-turretSpeed);
          } else {
            System.out.println("Left Limit");
            m_turret.stopTurret();
          }
       }
      }
    } else {
      m_turretState= reversed ? TurretStates.REVERSEDSEARCH : TurretStates.SEARCH;
      System.out.println("No Target Available");
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
        trackHub(m_flipTrack);
        break;
      case FIXEDLEFT:
        break;
      case FIXEDRIGHT:
        break;
      case SEARCH: 
        turretSearch(false);
        break;
      case REVERSEDSEARCH:
        turretSearch(true);
        break;
      case END:
        returnValue=true;
        break;
    } 
    return returnValue;
  }
}

