package frc.robot.drive.swerve;

/**
 * Interface for all Invictus 3593 drive systems to implement. 
 */
public interface IDriveSystem {
    void DriveTank(double left, double right);

    void DriveHolonomic(double STR, double FWD, double RCW);

    void DriveHolonomic(double heading, double STR, double FWD, double RCW);

    void StopAllMotors();
}