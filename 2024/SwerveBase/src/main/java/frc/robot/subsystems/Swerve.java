package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import java.io.File;
import java.util.function.DoubleSupplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import frc.robot.swervelib.SwerveDrive;
import frc.robot.swervelib.parser.SwerveParser;
import frc.robot.swervelib.telemetry.SwerveDriveTelemetry;
import frc.robot.swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

public class Swerve extends SubsystemBase {

  private final SwerveDrive swerve;

  private SlewRateLimiter translationLimiter = new SlewRateLimiter(Units.feetToMeters(14.5));
  private SlewRateLimiter strafeLimiter = new SlewRateLimiter(Units.feetToMeters(14.5));
  private SlewRateLimiter rotationLimiter = new SlewRateLimiter(Units.feetToMeters(14.5));

  /** Subsystem class for the swerve drive. */
  public Swerve() {
  // TODO: ReplaceTuning Mode for Logging verbosity
    // if (SpartanEntryManager.isTuningMode()) {
      SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;
  //  } else {
  //    SwerveDriveTelemetry.verbosity = TelemetryVerbosity.LOW;
  //  }

    try {
      swerve =
          new SwerveParser(new File(Filesystem.getDeployDirectory(), "swerve")).createSwerveDrive();
    } catch (Exception e) {
      throw new RuntimeException(e);
    }
      //KK Add AutoBuilder
      AutoBuilder.configureHolonomic(
      this::getPose, 
      this::resetOdometry, 
      this::getRobotVelocity, 
      this::setChassisSpeeds, 
      Constants.Swerve.pathFollowerConfig, 
      this
    );

  }

  public Command drive(
      DoubleSupplier translationSup, DoubleSupplier strafeSup, DoubleSupplier rotationSup) {
    return run(() -> {
          double translationVal =
              translationLimiter.calculate(
                  MathUtil.applyDeadband(
                      translationSup.getAsDouble(), Constants.GeneralConstants.swerveDeadband));
          double strafeVal =
              strafeLimiter.calculate(
                  MathUtil.applyDeadband(
                      strafeSup.getAsDouble(), Constants.GeneralConstants.swerveDeadband));
          double rotationVal =
              rotationLimiter.calculate(
                  MathUtil.applyDeadband(
                      rotationSup.getAsDouble(), Constants.GeneralConstants.swerveDeadband));
          SmartDashboard.putNumber("translationvalue",translationVal);
          SmartDashboard.putNumber("strafevalue",strafeVal);
      
          drive(
              new Translation2d(translationVal, strafeVal)
                  .times(swerve.swerveController.config.maxSpeed),
              rotationVal * swerve.swerveController.config.maxAngularVelocity,
              true,
              false);
        })
        .withName("TeleopSwerve");
  }

  public void drive(
      Translation2d translationVal, double rotationVal, boolean fieldRelative, boolean openLoop) {
    swerve.drive(translationVal, rotationVal, fieldRelative, openLoop);
  }

  public void setChassisSpeeds(ChassisSpeeds speeds) {
    swerve.setChassisSpeeds(speeds);
  }

  public void setMotorBrake(boolean brake) {
    swerve.setMotorIdleMode(brake);
  }

  public void zeroGyro() {
    swerve.zeroGyro();
  }

  public void lock() {
    swerve.lockPose();
  }

  public double getYaw() {
    return swerve.getYaw().getDegrees();
  }

  public double getPitch() {
    return swerve.getPitch().getDegrees();
  }

  public void resetOdometry(Pose2d pose) {
    swerve.resetOdometry(pose);
  }

  public Pose2d getPose() {
    return swerve.getPose();
  }
  public ChassisSpeeds getRobotVelocity(){
    return swerve.getRobotVelocity();
  }
  public Command getPathCommand(String pathName){
    try{
    PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);
    System.out.println("Path File "+ pathName);
    if(path!=null){ 
       System.out.println("path" + path.toString());
    }
    //   return null;
    return AutoBuilder.followPathWithEvents(path);
  }
  catch(Exception e){
     System.out.println(e.toString());
     return null;
  }
  }
  @Override
  public void periodic() {
    swerve.updateOdometry();
   
  }
}
