package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.SwerveDriveGB;
import frc.robot.classes.Limelight;


public class GyroResetCommand extends CommandBase {
   
 // private final DrivetrainSubsystem m_drive;
  private final SwerveDriveGB m_drive; 
 private final Limelight m_limelight;
  public GyroResetCommand(SwerveDriveGB subsystem, Limelight limelight) {
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