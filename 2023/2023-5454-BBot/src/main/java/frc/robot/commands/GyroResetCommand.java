package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.classes.Limelight;


public class GyroResetCommand extends CommandBase {
   
  private final SwerveSubsystem m_drive;
  private final Limelight m_limelight;
  private boolean m_isFinished=false;
  public GyroResetCommand(SwerveSubsystem subsystem, Limelight limelight) {
    m_drive = subsystem;
    m_limelight=limelight;
    addRequirements(m_drive);
  }

  @Override
  public void execute() {

    System.out.println("Resetting Gyro");
    m_limelight.turnLEDOff();
     m_drive.resetGyroscope();
     m_limelight.turnLEDOn();
     m_limelight.turnLEDOff();
     m_limelight.turnLEDOn();
     m_limelight.turnLEDOff();
     m_limelight.turnLEDOn();
     System.out.println("Resetting Gyro Complete");
     m_isFinished=true;
    }
    // Returns true when the command should end.
 
    @Override
    public boolean isFinished() {
      return m_isFinished;
    }
    
}