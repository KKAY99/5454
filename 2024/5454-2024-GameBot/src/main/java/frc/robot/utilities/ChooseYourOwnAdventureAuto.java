package frc.robot.utilities;
import java.util.List;

import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.Constants;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.AutoConstants.AutonomousRoutines;
import frc.robot.Constants.AutoConstants.StartingLocations;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.commands.AutoDoNothingCommand;
import frc.robot.commands.ShootCommand;
import frc.robot.commands.SmartShooter;
import frc.robot.commands.TurretCommand;
import frc.robot.commands.TurretPosCommand;
import frc.robot.commands.SmartShooter;
import frc.robot.commands.IntakeToggleCommand;

public class ChooseYourOwnAdventureAuto {
  private Swerve m_swerve;
  private ShooterSubsystem m_shooter;
  private IntakeSubsystem m_intake;
  private Limelight m_limeLight;
  private TurretSubsystem m_turret;

  private Command m_shoot1;
  private Command m_shoot2;
  private Command m_shoot3;
  private Command m_shoot4;
  private Command m_shoot5;

  private Command m_startIntake1;
  private Command m_startIntake2;
  private Command m_startIntake3;
  private Command m_startIntake4;
  private Command m_startIntake5;

  private Command m_stopIntake1;
  private Command m_stopIntake2;
  private Command m_stopIntake3;
  private Command m_stopIntake4;
  private Command m_stopIntake5;

  private Command m_turretSet1;
  private Command m_turretSet2;
  private Command m_turretSet3;
  private Command m_turretSet4;
  private Command m_turretSet5;

  private Command m_path1;
  private Command m_path2;
  private Command m_path3;
  private Command m_path4;
  private Command m_path5;

  private Command m_intakeWayPoint1;
  private Command m_intakeWayPoint2;
  private Command m_intakeWayPoint3;
  private Command m_intakeWayPoint4;
  private Command m_intakeWayPoint5;

  private Command m_shotWayPoint1;
  private Command m_shotWayPoint2;
  private Command m_shotWayPoint3;
  private Command m_shotWayPoint4;
  private Command m_shotWayPoint5;

  public ChooseYourOwnAdventureAuto(Swerve swerveDrive,ShooterSubsystem shooter,IntakeSubsystem intake,TurretSubsystem turret,Limelight limelight){
    m_swerve=swerveDrive;
    m_shooter=shooter;
    m_intake=intake;
    m_turret=turret;
    m_limeLight=limelight;

    CreateCommands();
  }

  public void CreateCommands(){
    m_shoot1=new SmartShooter(m_shooter,m_turret,m_swerve,m_limeLight,m_intake,false,false);
    m_shoot2=new SmartShooter(m_shooter,m_turret,m_swerve,m_limeLight,m_intake,false,false);
    m_shoot3=new SmartShooter(m_shooter,m_turret,m_swerve,m_limeLight,m_intake,false,false);
    m_shoot4=new SmartShooter(m_shooter,m_turret,m_swerve,m_limeLight,m_intake,false,false);
    m_shoot5=new SmartShooter(m_shooter,m_turret,m_swerve,m_limeLight,m_intake,false,false);

    m_startIntake1=new IntakeToggleCommand(m_intake,Constants.IntakeConstants.intakeSpeed,true);
    m_startIntake2=new IntakeToggleCommand(m_intake,Constants.IntakeConstants.intakeSpeed,true);
    m_startIntake3=new IntakeToggleCommand(m_intake,Constants.IntakeConstants.intakeSpeed,true);
    m_startIntake4=new IntakeToggleCommand(m_intake,Constants.IntakeConstants.intakeSpeed,true);
    m_startIntake5=new IntakeToggleCommand(m_intake,Constants.IntakeConstants.intakeSpeed,true);

    m_stopIntake1=new IntakeToggleCommand(m_intake,Constants.IntakeConstants.intakeSpeed,true);
    m_stopIntake2=new IntakeToggleCommand(m_intake,Constants.IntakeConstants.intakeSpeed,true);
    m_stopIntake3=new IntakeToggleCommand(m_intake,Constants.IntakeConstants.intakeSpeed,true);
    m_stopIntake4=new IntakeToggleCommand(m_intake,Constants.IntakeConstants.intakeSpeed,true);
    m_stopIntake5=new IntakeToggleCommand(m_intake,Constants.IntakeConstants.intakeSpeed,true);
  }

  public void SetPathCommands(AutoPose2D pose1,AutoPose2D pose2,AutoPose2D pose3,AutoPose2D pose4,AutoPose2D pose5){
    if(pose1!=null){
      m_path1=m_swerve.createPathCommand(CreateAutoPath(m_swerve.getPose(),pose1.getPathPos()));
      m_intakeWayPoint1=m_swerve.createPathCommand(CreateAutoPath(m_swerve.getPose(),pose1.getIntakePos()));
      m_shotWayPoint1=m_swerve.createPathCommand(CreateAutoPath(m_swerve.getPose(),pose1.getShotPos()));
      m_turretSet1=new TurretPosCommand(m_turret,pose1.getIntakeRotatePose(),Constants.TurretConstants.turretMoveTimeOut,Constants.TurretConstants.deadband);
    }else{
      m_path1=new AutoDoNothingCommand();
      m_intakeWayPoint1=new AutoDoNothingCommand();
      m_shotWayPoint1=new AutoDoNothingCommand();
      m_turretSet1=new AutoDoNothingCommand();
    }
    if(pose2!=null){
      m_path2=m_swerve.createPathCommand(CreateAutoPath(m_swerve.getPose(),pose2.getPathPos()));
      m_intakeWayPoint2=m_swerve.createPathCommand(CreateAutoPath(m_swerve.getPose(),pose2.getIntakePos()));
      m_shotWayPoint2=m_swerve.createPathCommand(CreateAutoPath(m_swerve.getPose(),pose2.getShotPos()));
      m_turretSet2=new TurretPosCommand(m_turret,pose2.getIntakeRotatePose(),Constants.TurretConstants.turretMoveTimeOut,Constants.TurretConstants.deadband);
    }else{
      m_path2=new AutoDoNothingCommand();
      m_intakeWayPoint2=new AutoDoNothingCommand();
      m_shotWayPoint2=new AutoDoNothingCommand();
      m_turretSet2=new AutoDoNothingCommand();
    }
    if(pose3!=null){
      m_path3=m_swerve.createPathCommand(CreateAutoPath(m_swerve.getPose(),pose3.getPathPos()));
      m_intakeWayPoint3=m_swerve.createPathCommand(CreateAutoPath(m_swerve.getPose(),pose3.getIntakePos()));
      m_shotWayPoint3=m_swerve.createPathCommand(CreateAutoPath(m_swerve.getPose(),pose3.getShotPos()));
      m_turretSet3=new TurretPosCommand(m_turret,pose3.getIntakeRotatePose(),Constants.TurretConstants.turretMoveTimeOut,Constants.TurretConstants.deadband);
    }else{
      m_path3=new AutoDoNothingCommand();
      m_intakeWayPoint3=new AutoDoNothingCommand();
      m_shotWayPoint3=new AutoDoNothingCommand();
      m_turretSet3=new AutoDoNothingCommand();
    }
    if(pose4!=null){
      m_path4=m_swerve.createPathCommand(CreateAutoPath(m_swerve.getPose(),pose4.getPathPos()));
      m_intakeWayPoint4=m_swerve.createPathCommand(CreateAutoPath(m_swerve.getPose(),pose4.getIntakePos()));
      m_shotWayPoint4=m_swerve.createPathCommand(CreateAutoPath(m_swerve.getPose(),pose4.getShotPos()));
      m_turretSet4=new TurretPosCommand(m_turret,pose4.getIntakeRotatePose(),Constants.TurretConstants.turretMoveTimeOut,Constants.TurretConstants.deadband);
    }else{
      m_path4=new AutoDoNothingCommand();
      m_intakeWayPoint4=new AutoDoNothingCommand();
      m_shotWayPoint4=new AutoDoNothingCommand();
      m_turretSet4=new AutoDoNothingCommand();
    }
    if(pose5!=null){
      m_path5=m_swerve.createPathCommand(CreateAutoPath(m_swerve.getPose(),pose5.getPathPos()));
      m_intakeWayPoint5=m_swerve.createPathCommand(CreateAutoPath(m_swerve.getPose(),pose5.getIntakePos()));
      m_shotWayPoint5=m_swerve.createPathCommand(CreateAutoPath(m_swerve.getPose(),pose5.getShotPos()));
      m_turretSet5=new TurretPosCommand(m_turret,pose5.getIntakeRotatePose(),Constants.TurretConstants.turretMoveTimeOut,Constants.TurretConstants.deadband);
    }else{
      m_path5=new AutoDoNothingCommand();
      m_intakeWayPoint5=new AutoDoNothingCommand();
      m_shotWayPoint5=new AutoDoNothingCommand();
      m_turretSet5=new AutoDoNothingCommand();
    } 
  }

  public PathPlannerPath CreateAutoPath(Pose2d startPose,Pose2d desiredPose){
    List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(
      startPose,
      desiredPose
    );

    PathPlannerPath newPath = new PathPlannerPath(
          bezierPoints,
          new PathConstraints(3.0, 3.0, 2 * Math.PI, 4 * Math.PI), // The constraints for this path. If using a differential drivetrain, the angular constraints have no effect.
          new GoalEndState(0.0, Rotation2d.fromDegrees(0)) // Goal end state. You can set a holonomic rotation here. If using a differential drivetrain, the rotation will have no effect.
    );

    return newPath;
  }

  public Command CreateAutoCommand(AutoPose2D pose1,AutoPose2D pose2,AutoPose2D pose3,AutoPose2D pose4,AutoPose2D pose5,boolean shouldShootFinalNote){
    SequentialCommandGroup newSequentialCommand=new SequentialCommandGroup(null);
    SetPathCommands(pose1,pose2,pose3,pose4,pose5);

    if(pose1!=null){
      newSequentialCommand.addCommands(m_intakeWayPoint1,m_turretSet1,m_startIntake1,m_path1,m_stopIntake1,m_shotWayPoint1,m_shoot1);
    }
    if(pose2!=null){
      newSequentialCommand.addCommands(m_intakeWayPoint2,m_turretSet2,m_startIntake2,m_path2,m_stopIntake2,m_shotWayPoint2,m_shoot2);
    }
    if(pose3!=null){
      newSequentialCommand.addCommands(m_intakeWayPoint3,m_turretSet3,m_startIntake3,m_path3,m_stopIntake3,m_shotWayPoint3,m_shoot3);
    }
    if(pose4!=null){
      newSequentialCommand.addCommands(m_intakeWayPoint4,m_turretSet4,m_startIntake4,m_path4,m_stopIntake4,m_shotWayPoint4,m_shoot4);
    }
    if(pose5!=null){
      Command shootFinalNote=null;
      
      if(shouldShootFinalNote){
        shootFinalNote=m_shoot5;
      }else{
        shootFinalNote=new AutoDoNothingCommand();
      }

      newSequentialCommand.addCommands(m_intakeWayPoint5,m_turretSet5,m_startIntake5,m_path5,m_stopIntake5,m_shotWayPoint5,shootFinalNote);
    }

    Command newCommand=newSequentialCommand;

    return newCommand;
  }
}


