package frc.robot.commands;

import static edu.wpi.first.units.Units.*;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.Constants.VisionConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.LimelightHelpers.RawFiducial;
import edu.wpi.first.math.controller.PIDController;
class PIDControllerConfigurable extends PIDController {
  public PIDControllerConfigurable(double kP, double kI, double kD) {
      super(kP, kI, kD);
  }
  
  public PIDControllerConfigurable(double kP, double kI, double kD, double tolerance) {
      super(kP, kI, kD);
      this.setTolerance(tolerance);
  }
}
public class AlignCommand extends Command {
  private final CommandSwerveDrivetrain m_drivetrain;
  private final VisionSubsystem m_Limelight;

  //private static final PIDControllerConfigurable rotationalPidController = new PIDControllerConfigurable(0.05000, 0.000000, 0.001000, 0.01);
  private static PIDControllerConfigurable rotationalPidController = new PIDControllerConfigurable(VisionConstants.ROTATE_P, VisionConstants.ROTATE_I, VisionConstants.ROTATE_D, VisionConstants.TOLERANCE);
  //private static final PIDControllerConfigurable xPidController = new PIDControllerConfigurable(0.400000, 0.000000, 0.000600, 0.01);
  private static final PIDControllerConfigurable xPidController = new PIDControllerConfigurable(VisionConstants.MOVE_P, VisionConstants.MOVE_I, VisionConstants.MOVE_D, VisionConstants.TOLERANCE);

  

  private static final SwerveRequest.RobotCentric alignRequest = new SwerveRequest.RobotCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);
  private static final SwerveRequest.Idle idleRequest = new SwerveRequest.Idle();
  private static int tagID = -1;
  
  public double rotationalRate = 0;
  public double velocityX = 0;

  //use whatever fiducial is closest
  public AlignCommand(CommandSwerveDrivetrain drivetrain, VisionSubsystem limelight) {
    this.m_drivetrain = drivetrain;
    this.m_Limelight = limelight;
    addRequirements(m_Limelight);
  }

  //Overload for specific april tag by id

  public AlignCommand(CommandSwerveDrivetrain drivetrain, VisionSubsystem limelight, int ID) throws IllegalArgumentException{
    this.m_drivetrain = drivetrain;
    this.m_Limelight = limelight;
    if (ID<0){throw new IllegalArgumentException("april tag id cannot be negative");}
    tagID = ID;
    addRequirements(m_Limelight);
  }

  //maybe set tagid by closest here instead of each execution to keep same tag
  @Override
  public void initialize() {}

  @Override
  public void execute() {

    rotationalPidController = new PIDControllerConfigurable(SmartDashboard.getNumber("Rotate P", 0.0), VisionConstants.ROTATE_I, SmartDashboard.getNumber("Rotate D", 0.0), VisionConstants.TOLERANCE);
    
    RawFiducial fiducial; //Tracked fiducual 

    try {
      if (tagID==-1){
        fiducial = m_Limelight.getFiducialWithId(m_Limelight.getClosestFiducial().id);
      }
      else{
        fiducial = m_Limelight.getFiducialWithId(tagID);
      }
       

      rotationalRate = rotationalPidController.calculate(2*fiducial.txnc, 0.0) * RotationsPerSecond.of(0.75).in(RadiansPerSecond) * -0.1; // Max speed is 90 percnet of max rotate
      
      final double velocityX = xPidController.calculate(fiducial.distToRobot, 0.3) * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond) * 0.6; //Max speed is 70 percnet of max drive
        
      if (rotationalPidController.atSetpoint() && xPidController.atSetpoint()) { //At target dist
        this.end(false);
      }

      SmartDashboard.putNumber("txnc", fiducial.txnc);
      SmartDashboard.putNumber("distToRobot", fiducial.distToRobot);
      SmartDashboard.putNumber("rotationalPidController", rotationalRate);
      SmartDashboard.putNumber("xPidController", velocityX);

      m_drivetrain.setControl(
          alignRequest.withRotationalRate(-rotationalRate).withVelocityX(-velocityX));

    } catch (VisionSubsystem.NoSuchTargetException nste) { 
      System.out.println("No apriltag found");
        m_drivetrain.setControl(
          alignRequest.withRotationalRate(-1.5*rotationalRate).withVelocityX(-1.5*velocityX)); //Continue moving after losing sight temporarily 
      }
      
    }
  

  @Override
  public boolean isFinished() {
    return rotationalPidController.atSetpoint() && xPidController.atSetpoint();
  }

  @Override
  public void end(boolean interrupted) {
    m_drivetrain.applyRequest(() -> idleRequest);
  }
}