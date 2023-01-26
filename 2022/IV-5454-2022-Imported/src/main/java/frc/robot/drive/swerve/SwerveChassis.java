package frc.robot.drive.swerve;

import java.util.Arrays;
import java.util.Collections;
import java.util.Dictionary;
import java.util.Hashtable;



public class SwerveChassis implements IDriveSystem {
    public Wheel FrontRightWheel, FrontLeftWheel, RearRightWheel, RearLeftWheel;
    private double TrackWidth, Wheelbase;

    public SwerveChassis(WheelMap frontRightWheelMap, WheelMap frontLeftWheelMap, WheelMap rearRightWheelMap, WheelMap rearLeftWheelMap, PidConstants rotatorPidConstants) throws Exception {
        FrontRightWheel = new Wheel(
            "front-right",
            frontRightWheelMap.DriveMotorChannel, 
            frontRightWheelMap.RotatorMotorChannel, 
            frontRightWheelMap.EncoderChannel, 
            frontRightWheelMap.EncoderOffset, 
            rotatorPidConstants);
        FrontRightWheel.SetDriveInverted(true);

        FrontLeftWheel = new Wheel(
            "front-left",
            frontLeftWheelMap.DriveMotorChannel, 
            frontLeftWheelMap.RotatorMotorChannel, 
            frontLeftWheelMap.EncoderChannel, 
            frontLeftWheelMap.EncoderOffset, 
            rotatorPidConstants);
        FrontLeftWheel.SetDriveInverted(false);

        RearRightWheel = new Wheel(
            "rear-right",
            rearRightWheelMap.DriveMotorChannel, 
            rearRightWheelMap.RotatorMotorChannel, 
            rearRightWheelMap.EncoderChannel, 
            rearRightWheelMap.EncoderOffset, 
            rotatorPidConstants);
        RearRightWheel.SetDriveInverted(true);

        RearLeftWheel = new Wheel(
            "rear-left",
            rearLeftWheelMap.DriveMotorChannel, 
            rearLeftWheelMap.RotatorMotorChannel, 
            rearLeftWheelMap.EncoderChannel, 
            rearLeftWheelMap.EncoderOffset, 
            rotatorPidConstants);
        RearRightWheel.SetDriveInverted(true);

        Wheelbase = RobotMap.Wheelbase;
        TrackWidth = RobotMap.TrackWidth;

        MultiLogger.Debug("[DRIVE] Successfully started");
    }

    /**
     * Sets all wheels to 0°
     */
    public void HomeWheels() {
        FrontLeftWheel.SetAngle(0);
        FrontRightWheel.SetAngle(0);
        RearLeftWheel.SetAngle(0);
        RearRightWheel.SetAngle(0);
    }

    /**
     * Stop all drive motors in this DriveSystem
     */
    public void StopAllMotors() {
        // Stop all motors
        FrontRightWheel.StopAllMotors();
        FrontLeftWheel.StopAllMotors();
        RearRightWheel.StopAllMotors();
        RearLeftWheel.StopAllMotors();
    }

    /**
     * Robot-centric drive with 2 strafe axes and 1 rotation axis
     * @param RCW
     * @param STR
     * @param FWD
     */
    public void DriveHolonomic(double RCW, double STR, double FWD) {
        DriveHolonomic(0, RCW, STR, FWD);
    }

    /**
     * Field-centric drive with 2 strafe axes and 1 rotation axis
     * @param gyroAngle 0-360 clockwise gyro angle
     * @param RCW -1 - 1 rotation coefficient
     * @param STR -1 - 1 X-axis strafe control
     * @param FWD -1 - 1 Y-axis strafe control
     */
    public void DriveHolonomic(double gyroAngle, double RCW, double STR, double FWD) {
        FWD *= -1;

        // Adjust vectors for gyro angle
        double gyroAngleRadians = Math.toRadians(gyroAngle);
        double adjustedFWD = (FWD * Math.cos(gyroAngleRadians)) + (STR * Math.sin(gyroAngleRadians));
        STR = (-FWD * Math.sin(gyroAngleRadians)) + (STR * Math.cos(gyroAngleRadians));
        FWD = adjustedFWD;

        double wheelBase = Wheelbase;
        double trackWidth = TrackWidth;
        double radius = Math.sqrt(Math.pow(wheelBase, 2) + Math.pow(trackWidth, 2));

        double A = STR - RCW * (wheelBase / radius);
        double B = STR + RCW * (wheelBase / radius);
        double C = FWD - RCW * (trackWidth / radius);
        double D = FWD + RCW * (trackWidth / radius);

        // Front right
        double frontRightSpeed = Math.sqrt(Math.pow(B, 2) + Math.pow(C, 2));
        double frontRightAngle = Math.atan2(B, C) * 180/Math.PI;

        // Front left
        double frontLeftSpeed = Math.sqrt(Math.pow(B, 2) + Math.pow(D, 2));
        double frontLeftAngle = Math.atan2(B, D) * 180/Math.PI;

        // Rear right
        double rearRightSpeed = Math.sqrt(Math.pow(A, 2) + Math.pow(C, 2));
        double rearRightAngle = Math.atan2(A, C) * 180/Math.PI;

        // Rear left
        double rearLeftSpeed = Math.sqrt(Math.pow(A, 2) + Math.pow(D, 2));
        double rearLeftAngle = Math.atan2(A, D) * 180/Math.PI;

        // If, after calculating the 4 wheel speeds, any of them is greater than 1, then divide all the wheel speeds by the largest value.
        double maxSpeed = 1;
        if (frontRightSpeed > maxSpeed || frontLeftSpeed > maxSpeed || rearRightSpeed > maxSpeed || rearLeftSpeed > maxSpeed) {
            Double[] speeds = new Double[] { frontRightSpeed, frontLeftSpeed, rearRightSpeed, rearLeftSpeed };
            double maxValue = Collections.max(Arrays.asList(speeds));

            frontRightSpeed /= maxValue;
            frontLeftSpeed /= maxValue;
            rearRightSpeed /= maxValue;
            rearLeftSpeed /= maxValue;
        }

        // Calculated angles range from -180° to 180°
        frontRightAngle = Utilities.ClampAngle(frontRightAngle);
        frontLeftAngle = Utilities.ClampAngle(frontLeftAngle);
        rearRightAngle = Utilities.ClampAngle(rearRightAngle);
        rearLeftAngle = Utilities.ClampAngle(rearLeftAngle);

        FrontRightWheel.SetWheel(frontRightSpeed, frontRightAngle);
        FrontLeftWheel.SetWheel(frontLeftSpeed, frontLeftAngle);
        RearRightWheel.SetWheel(rearRightSpeed, rearRightAngle);
        RearLeftWheel.SetWheel(rearLeftSpeed, rearLeftAngle);
    }

    /**
     * Locks wheels to 0° and drives wheel speeds like a differential chassis
     * @param left Left-side speed
     * @param right Right-side speed
     */
    public void DriveTank(double left, double right) {
        FrontRightWheel.SetWheel(right, 0);
        FrontLeftWheel.SetWheel(left, 0);
        RearRightWheel.SetWheel(right, 0);
        RearLeftWheel.SetWheel(left, 0);
    }

    public Dictionary<String, Object> GetWheelStats() {
        Dictionary<String, Object> wheels = new Hashtable<String, Object>();

        wheels.put("frontLeft", FrontLeftWheel.GetAllStats());
        wheels.put("frontRight", FrontRightWheel.GetAllStats());
        wheels.put("rearLeft", RearLeftWheel.GetAllStats());
        wheels.put("rearRight", RearRightWheel.GetAllStats());

        return wheels;
    }
}