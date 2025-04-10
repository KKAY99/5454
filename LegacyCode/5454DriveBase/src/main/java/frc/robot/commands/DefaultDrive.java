package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.Constants;
public class DefaultDrive extends CommandBase {
  private final DriveSubsystem m_drive;
  private final DoubleSupplier m_left;
  private final DoubleSupplier m_right;

  /**
   * Creates a new DefaultDrive.
   *
   * @param subsystem The drive subsystem this command wil run on.
   * @param forward The control input for driving left
   * @param rotation The control input for driving right
   */
  public DefaultDrive(DriveSubsystem subsystem, DoubleSupplier left, DoubleSupplier right) {
    m_drive = subsystem;
    m_left = right;
    m_right = left;
    addRequirements(m_drive);
  }

  @Override
  public void execute() {
   // System.out.println('Speeds' +  m_left.getAsDouble()  + m_right.getAsDouble()
   
   // m_drive.tankDrive(m_left.getAsDouble() * Constants.kSpeedMultiplier, m_right.getAsDouble()* Constants.kSpeedMultiplier);
   m_drive.arcadeDrive(m_left.getAsDouble() * Constants.kSpeedMultiplier, m_right.getAsDouble()* Constants.kSpeedMultiplier);
  
  }
} 