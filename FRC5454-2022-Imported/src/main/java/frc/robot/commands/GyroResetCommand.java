package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.classes.Limelight;
import frc.robot.common.Utilities;

public class GyroResetCommand extends CommandBase {
   
  private final DrivetrainSubsystem m_drive;
  private final Limelight m_limelight;
  public GyroResetCommand(DrivetrainSubsystem subsystem, Limelight limelight) {
    m_drive = subsystem;
    m_limelight=limelight;
    addRequirements(m_drive);
  }

  @Override
  public void execute() {
     m_limelight.turnLEDOff();
     m_drive.resetGyroscope();
     m_limelight.turnLEDOn();
     m_limelight.turnLEDOff();
     m_limelight.turnLEDOn();
     m_limelight.turnLEDOff();
     m_limelight.turnLEDOn();
     
  }
}