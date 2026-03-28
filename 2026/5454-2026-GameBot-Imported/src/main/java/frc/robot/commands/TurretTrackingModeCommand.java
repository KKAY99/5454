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

@SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})  

public class TurretTrackingModeCommand extends Command {

  private TurretSubsystemPots m_turret;
  private CommandSwerveDrivetrain m_drive;
  private Limelight m_limelight;
  private TurretStates m_turretState;
  private TurretStates m_initState;

  public TurretTrackingModeCommand(TurretSubsystemPots turret,CommandSwerveDrivetrain drive,TurretStates state,Limelight limelight) {
    m_turret=turret;    
    m_drive=drive;
    m_turretState=state;
    m_initState=state;
    m_limelight=limelight;
    addRequirements(m_turret);
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //filter on front of hubs

  }
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
   
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("Ending Turret Track");
    m_turret.stopTurret();
    
  }

  
  private void trackHub(TurretUtil.TargetType target ){
    target=TargetType.HUB;
  
     double turretAngleTarget=TurretUtil.get5454TurretAngle(m_drive.getPose2d(),target);
  
     SmartDashboard.putNumber("Turret Util Target Angle",turretAngleTarget);
     double targetPos=m_turret.getTargetMotorPosition(turretAngleTarget);
     SmartDashboard.putNumber("Turret Util Target Pos",targetPos); 
     m_turret.moveMotor(targetPos);

  
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    boolean returnValue=false;
    return returnValue;
  }
}

