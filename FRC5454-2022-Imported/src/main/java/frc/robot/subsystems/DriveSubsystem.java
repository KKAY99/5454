
package frc.robot.subsystems;

//import edu.wpi.first.wpilibj.Encoder;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.classes.SwerveDriveGB;
import com.kauailabs.navx.frc.AHRS;

public class DriveSubsystem extends SubsystemBase {

  private SwerveDriveGB m_drive;

  public DriveSubsystem(AHRS ahrs) {
    m_drive = new SwerveDriveGB(ahrs);

  }

  public void drive(double fwd, double strafe, double rcw) {
    m_drive.drive(fwd, strafe, rcw, true);
    
  }

  public void move (double direction, double speed, double distance, boolean stopAtFalse)
{
  m_drive.move(direction,speed,distance,stopAtFalse);
}


}
