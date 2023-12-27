package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import java.util.function.DoubleSupplier;
import javax.lang.model.util.ElementScanner14;
import java.util.function.BooleanSupplier;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import frc.robot.subsystems.WPIDriveTrainSubsystem;
import frc.robot.common.Utilities;
import frc.robot.Constants;
import frc.robot.Constants.WPISwerve;
public class WPIDriveCommand extends CommandBase {
   
  private final WPIDriveTrainSubsystem m_WPIdrive;
  //private final SwerveSubsystem m_drive;
  private final DoubleSupplier m_drive_fwd;
  private final DoubleSupplier m_drive_strafe;
  private final DoubleSupplier m_drive_rcw;
  private final BooleanSupplier m_fieldMode;

 // priv//ate final SlewRateLimiter xLimiter;
 // private final SlewRateLimiter yLimiter;
  //private final SlewRateLimiter turnLimiter;

  private final double xyLimit=8;
  private final double rotLimit=2;


  private double initalHeading;

  public WPIDriveCommand(WPIDriveTrainSubsystem subsystem, DoubleSupplier drive_rcw, DoubleSupplier drive_fwd,
      DoubleSupplier drive_strafe,BooleanSupplier isFieldMode) {
    m_WPIdrive = subsystem;
    m_drive_fwd = drive_fwd;
    m_drive_strafe = drive_strafe;
    m_drive_rcw = drive_rcw;
    m_fieldMode=isFieldMode;
    
    //this.xLimiter=new SlewRateLimiter(xyLimit);
    //this.yLimiter=new SlewRateLimiter(xyLimit);
    //this.turnLimiter=new SlewRateLimiter(rotLimit);

    initalHeading=m_WPIdrive.newHeading();

    addRequirements(m_WPIdrive);
  }

  @Override
  public void initialize(){
  }

  @Override
  public void execute() {
    double fwdSpeed=m_drive_fwd.getAsDouble();
    double strafeSpeed=m_drive_fwd.getAsDouble();
    double rotSpeed=m_drive_rcw.getAsDouble();

    fwdSpeed=Math.abs(fwdSpeed)>0.05?fwdSpeed:0.0;
    strafeSpeed=Math.abs(strafeSpeed)>0.05?strafeSpeed:0.0;
    rotSpeed=Math.abs(rotSpeed)>0.05?rotSpeed:0.0;

    //fwdSpeed=yLimiter.calculate(fwdSpeed)*Constants.WPISwerve.physicalMaxSpeedMetersPerSecond;
    //strafeSpeed=xLimiter.calculate(strafeSpeed)*Constants.WPISwerve.physicalMaxSpeedMetersPerSecond;
    //rotSpeed=turnLimiter.calculate(rotSpeed)*Constants.WPISwerve.maxSwerveAngularSpeedRadianPerSecond;
   
    rotSpeed=Math.abs(rotSpeed)>0.05?rotSpeed:0.0;
    rotSpeed*=1;

    //System.out.println("Forward Speed " + fwdSpeed);
    //System.out.println("Strafe Speed " + strafeSpeed);
    //System.out.println("Rot Speed " + rotSpeed);
    
    if(rotSpeed>Constants.WPISwerve.physicalMaxAngularSpeedRadiansPerSecond){
      rotSpeed=Constants.WPISwerve.physicalMaxAngularSpeedRadiansPerSecond;
    }else if(rotSpeed< -Constants.WPISwerve.physicalMaxAngularSpeedRadiansPerSecond){
      rotSpeed= -Constants.WPISwerve.physicalMaxAngularSpeedRadiansPerSecond;
    }

    ChassisSpeeds chassisSpeeds;
    chassisSpeeds=ChassisSpeeds.fromFieldRelativeSpeeds(strafeSpeed,fwdSpeed,rotSpeed,
                                      Rotation2d.fromDegrees(m_WPIdrive.getRobotDegrees()));
    //System.out.println("FieldRelativeY"+chassisSpeeds.vxMetersPerSecond);

    SwerveModuleState[] moduleStates=m_WPIdrive.m_driveKinematics.toSwerveModuleStates(chassisSpeeds);
    //System.out.println("ModuleStates1 "+moduleStates[0].angle.getDegrees());
   // System.out.println("ModuleStates2 "+moduleStates[1].angle.getDegrees());
    //System.out.println("ModuleStates3 "+moduleStates[2].angle.getDegrees());
    //System.out.println("ModuleStates4 "+moduleStates[3].angle.getDegrees());
    m_WPIdrive.setModuleStates(moduleStates);
  }

  @Override
  public void end(boolean interrupted){
    m_WPIdrive.stopAllModules();
  }

  @Override
  public boolean isFinished(){
    return false;
  }
}