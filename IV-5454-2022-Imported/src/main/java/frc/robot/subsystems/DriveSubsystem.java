
package frc.robot.subsystems;

//import edu.wpi.first.wpilibj.Encoder;


import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.drive.swerve.RobotMap;
import frc.robot.drive.swerve.SwerveChassis;
import frc.robot.drive.swerve.MultiLogger;
import com.kauailabs.navx.frc.AHRS;

public class DriveSubsystem extends SubsystemBase {
  public SwerveChassis Chassis;
  public boolean VirtualHighGear = false;

  public DriveSubsystem(AHRS gyro) {
    try {
      Chassis = new SwerveChassis(
        RobotMap.FrontRightWheelMap, 
        RobotMap.FrontLeftWheelMap, 
        RobotMap.RearRightWheelMap,
        RobotMap.RearLeftWheelMap, 
        RobotMap.PID_Drive_SwerveRotatorConstants
      );
      
    } catch (Exception ex) {
     //  DriverStation.reportError("Error initializing drive system: " + ex.getMessage(), ex.getStackTrace());
    }
  }


  public void swerveDrive(double fwd,double strafe, double rcw){
    Chassis.DriveHolonomic(rcw, strafe, fwd);
    //m_drive.drive(fwd,strafe, rcw,true);
  }
  
  
  
  
}

