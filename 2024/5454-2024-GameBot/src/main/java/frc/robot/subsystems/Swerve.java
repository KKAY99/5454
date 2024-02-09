package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import java.io.File;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import frc.robot.swervelib.SwerveDrive;
import frc.robot.swervelib.parser.SwerveParser;
import frc.robot.swervelib.telemetry.SwerveDriveTelemetry;
import frc.robot.swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;
import frc.robot.utilities.FieldRelativeAccel;
import frc.robot.utilities.FieldRelativeSpeed;
import org.littletonrobotics.junction.Logger;

public class Swerve extends SubsystemBase {
  private final SwerveDrive swerve;
  private SwerveIO m_swerveIO;
  private SwerveIOInputsAutoLogged m_swerveAutoLogged=new SwerveIOInputsAutoLogged();

  private SlewRateLimiter translationLimiter = new SlewRateLimiter(Units.feetToMeters(14.5));
  private SlewRateLimiter strafeLimiter = new SlewRateLimiter(Units.feetToMeters(14.5));
  private SlewRateLimiter rotationLimiter = new SlewRateLimiter(Units.feetToMeters(14.5));
  
  //5454 Extensions based on 1706 SmartShooter from 2022
  private FieldRelativeSpeed m_fieldRelVel = new FieldRelativeSpeed();
  private FieldRelativeSpeed m_lastFieldRelVel = new FieldRelativeSpeed();
  private FieldRelativeAccel m_fieldRelAccel = new FieldRelativeAccel();;

  /** Subsystem class for the swerve drive. */
  public Swerve(SwerveIO swerveIO) {
    m_swerveIO=swerveIO;
  // TODO: ReplaceTuning Mode for Logging verbosity
    //  if (SpartanEntryManager.isTuningMode()) {
  //    SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;
  //  } else {
      SwerveDriveTelemetry.verbosity = TelemetryVerbosity.LOW;
  //  }

    try {
      swerve =
          new SwerveParser(new File(Filesystem.getDeployDirectory(), "swerve")).createSwerveDrive();
      //KK Add AutoBuilder
      AutoBuilder.configureHolonomic(
      this::getPose, 
      this::resetOdometry, 
      this::getRobotVelocity, 
      this::setChassisSpeeds, 
      Constants.Swerve.pathFollowerConfig,
      this.FlipPath(),
      this
    );

        } catch (Exception e) {
      throw new RuntimeException(e);
    }
   
  }

  public Command drive(
      DoubleSupplier translationSup, DoubleSupplier strafeSup, DoubleSupplier rotationSup, DoubleSupplier speedMultiplier) {
    return run(() -> {
          double translationVal =
              translationLimiter.calculate(
                  MathUtil.applyDeadband(
                       -translationSup.getAsDouble() * Math.max(speedMultiplier.getAsDouble(),Constants.DriveConstants.MinGasPedalSpeed)
                      ,Constants.DriveConstants.swerveDeadband));
          double strafeVal =
              strafeLimiter.calculate(
                  MathUtil.applyDeadband(
                      -strafeSup.getAsDouble() * Math.max(speedMultiplier.getAsDouble(),Constants.DriveConstants.MinGasPedalSpeed)
                      ,Constants.DriveConstants.swerveDeadband));
          double rotationVal =
              rotationLimiter.calculate(
                  MathUtil.applyDeadband(
                      -rotationSup.getAsDouble(), Constants.DriveConstants.swerveDeadband));
          SmartDashboard.putNumber("translationvalue",translationVal);
          SmartDashboard.putNumber("strafevalue",strafeVal);
          Logger.recordOutput("Swerve Translation", translationVal);
          Logger.recordOutput("Swerve Strafe Value",strafeVal);
          Logger.recordOutput("Rotation Value",rotationVal);
          drive(
              new Translation2d(translationVal, strafeVal)
                  .times(swerve.swerveController.config.maxSpeed),
              rotationVal * swerve.swerveController.config.maxAngularVelocity,
              true,
              false);
        })
        .withName("TeleopSwerve");
  }

  private BooleanSupplier FlipPath(){
    BooleanSupplier supplier;
    supplier=()->false;
    return supplier;
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
  public Rotation2d getGyro2D(){
    return swerve.getYaw();  // 5454 added for Smartshooter

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
  public Command createPathCommand(PathPlannerPath path){
    return AutoBuilder.followPath(path);
  }

  public Command getPathCommand(String pathName){
    try{
    PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);
    System.out.println("Path File "+ pathName);
    if(path!=null){ 
       System.out.println("path" + path.toString());
       System.out.println(path.getAllPathPoints().toString());
    }
    //   return null;
    return AutoBuilder.followPath(path);
  }
  catch(Exception e){
     System.out.println(e.toString());
     return null;
  }
  }

  public void AddVisionPose(Pose2d visionPose,double timeStamp,boolean soft,double trustWorthiness){
    swerve.addVisionMeasurement(visionPose,timeStamp,soft,trustWorthiness);
    //System.out.println("Robot Pose Swerve: "+getPose());
    //System.out.println("Robot Pose Limelight: "+visionPose);
  }

  public FieldRelativeSpeed getRelativeSpeed(){
    return m_fieldRelVel;
  }
  public FieldRelativeAccel getRelativeAccel(){
    return m_fieldRelAccel;
  }
  @Override
  public void periodic() {
    m_swerveIO.updateInputs(m_swerveAutoLogged);

    Logger.processInputs("Swerve",m_swerveAutoLogged);

    m_fieldRelVel = new FieldRelativeSpeed(swerve.getFieldVelocity(), swerve.getYaw());
    m_fieldRelAccel = new FieldRelativeAccel(m_fieldRelVel, m_lastFieldRelVel, Constants.kRobotLoopTime);
    m_lastFieldRelVel = m_fieldRelVel;
    
    swerve.updateOdometry();
    //5454 Update 
   // m_fieldRelVel = new FieldRelativeSpeed(getRobotVelolocity(),  );
   // m_fieldRelAccel = new FieldRelativeAccel(m_fieldRelVel, m_lastFieldRelVel, GlobalConstants.kLoopTime);
   // m_lastFieldRelVel = m_fieldRelVel;
  }
}
