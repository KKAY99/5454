package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.Constants;
public class DefaultDriveCommand extends CommandBase {
  private final DriveSubsystem m_drive;
  private final DoubleSupplier m_drive_fwd;
  private final DoubleSupplier m_drive_strafe;
  private final DoubleSupplier m_drive_rcw;

  /**
   * Creates a new DefaultDrive.
   *
   * @param subsystem The drive subsystem this command wil run on.
   * @param forward The control input for driving left
   * @param rotation The control input for driving right
   */
  public DefaultDriveCommand(DriveSubsystem subsystem, DoubleSupplier drive_fwd, DoubleSupplier drive_strafe,DoubleSupplier drive_rcw) {
    m_drive = subsystem;
    m_drive_fwd = drive_fwd;
    m_drive_strafe = drive_strafe;
    m_drive_rcw=drive_rcw;
    addRequirements(m_drive);
  }

  @Override
  public void execute() {
    System.out.println("FWD - " + m_drive_fwd.getAsDouble() + " STR - " + m_drive_strafe.getAsDouble() + "  RCW" + m_drive_rcw.getAsDouble());   
    m_drive.swerveDrive(m_drive_fwd.getAsDouble() * Constants.kSpeedMultiplier, m_drive_strafe.getAsDouble() *Constants.kSpeedMultiplier, m_drive_rcw.getAsDouble() *Constants.kSpeedMultiplier) ;
   
  }
} 