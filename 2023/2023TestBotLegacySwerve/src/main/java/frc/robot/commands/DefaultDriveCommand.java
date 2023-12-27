package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.SwerveDriveGB;
import frc.robot.common.Utilities;
import frc.robot.Constants;
import frc.robot.Constants.SwerveDriveGBConfig;
public class DefaultDriveCommand extends CommandBase {
   
  private final SwerveDriveGB m_drive;
  //private final SwerveSubsystem m_drive;
  private final DoubleSupplier m_drive_fwd;
  private final DoubleSupplier m_drive_strafe;
  private final DoubleSupplier m_drive_rcw;

  /**
   * Creates a new DefaultDrive.
   *
   * @param subsystem The drive subsystem this command wil run on.
   * @param forward   The control input for driving left
   * @param rotation  The control input for driving right
   */
  // FIXME SwerveSubsystem to DriveSubsystem
  public DefaultDriveCommand(SwerveDriveGB subsystem, DoubleSupplier drive_rcw, DoubleSupplier drive_fwd,
      DoubleSupplier drive_strafe) {
    m_drive = subsystem;
    m_drive_fwd = drive_fwd;
    m_drive_strafe = drive_strafe;
    m_drive_rcw = drive_rcw;
   addRequirements(m_drive);
  }

  @Override
  public void execute() {
    double forward = Utilities.deadband(m_drive_fwd.getAsDouble(),Constants.driveDeadband);
    // Square the forward stick
    forward = Math.copySign(Math.pow(forward, 2.0), forward);
    
    double strafe = Utilities.deadband(m_drive_strafe.getAsDouble());
    strafe = Utilities.deadband(strafe,Constants.driveDeadband);
    // Square the strafe stick
    strafe = Math.copySign(Math.pow(strafe, 2.0), strafe);
    //System.out.println("Straffing - " + strafe);
    double rotation = Utilities.deadband(m_drive_rcw.getAsDouble());
    rotation = Utilities.deadband(rotation,Constants.driveDeadband);
    // Square the rotation stick
    rotation = Math.copySign(Math.pow(rotation, 2.0), rotation);
    //System.out.println(forward + " -- " + strafe + " -- " + rotation);
    //MK2 Serve Drive
    // m_drive.drive(new Translation2d(forward, strafe), rotation, true);
    // 
    //m_drive.driveRobot(forward,strafe,rotation);
    m_drive.drive(rotation, forward,strafe,true);  
  }
}