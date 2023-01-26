package frc.robot.drive.swerve;

import java.util.Dictionary;
import java.util.Hashtable;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj.PIDController;

/**
 * Describes a swerve module in 3593's swerve drive chassis
 */
public class Wheel {
    public SwerveModuleEncoder HeadingEncoder;
    public CANSparkMax DriveMotor;
    public CANSparkMax RotatorMotor;
    public PIDController RotatorPidController;

    public Wheel(String wheelLocation, int driveMotorChannel, int turnMotorChannel, int encoderChannel, double rotatorOffset, PidConstants pidConstants) {
        // System.out.println("[SWERVE] Created new " + wheelLocation + " swerve wheel");
        // System.out.println("========== Drive Channel: " + driveMotorChannel);
        // System.out.println("========== Rotator Channel: " + turnMotorChannel);
        // System.out.println("========== Encoder Channel: " + encoderChannel);
        // System.out.println("========== Encoder Offset: " + rotatorOffset);
        
        DriveMotor = new CANSparkMax(driveMotorChannel, CANSparkMaxLowLevel.MotorType.kBrushless);
        DriveMotor.clearFaults();
        DriveMotor.setOpenLoopRampRate(1);
        // DriveMotor.setSmartCurrentLimit(20);
        // DriveMotor.setSecondaryCurrentLimit(30);
        DriveMotor.setIdleMode(IdleMode.kBrake);

        RotatorMotor = new CANSparkMax(turnMotorChannel, CANSparkMaxLowLevel.MotorType.kBrushless);
        RotatorMotor.clearFaults();
        RotatorMotor.setInverted(true);
        RotatorMotor.setOpenLoopRampRate(0.1);
        RotatorMotor.setSmartCurrentLimit(20);
        RotatorMotor.setSecondaryCurrentLimit(30);
        RotatorMotor.setIdleMode(IdleMode.kBrake);

        HeadingEncoder = new SwerveModuleEncoder(encoderChannel, rotatorOffset);

        RotatorPidController = new PIDController(pidConstants.kP, pidConstants.kI, pidConstants.kD, 0, HeadingEncoder, RotatorMotor, 0.02);
        RotatorPidController.setInputRange(-180, 180);
        RotatorPidController.setOutputRange(-1, 1);
        RotatorPidController.setPercentTolerance(2);
        RotatorPidController.setContinuous(true);
        RotatorPidController.setEnabled(true);
    }

    public void SetDriveInverted(boolean inverted) {
        DriveMotor.setInverted(inverted);
    }

    // Control
    
    public void StopAllMotors() {
        DriveMotor.stopMotor();
        RotatorMotor.stopMotor();
    }

    public void SetAngle(double angleSetpoint) {
        RotatorPidController.setSetpoint(angleSetpoint);
    }

    public void SetMagnitude(double magnitude) {
        double gainedSpeed = magnitude * RobotMap.DriveSpeedGain;
        double finalSpeed = Utilities.Deadband(gainedSpeed, RobotMap.DriveSpeedDeadband);
        DriveMotor.set(finalSpeed);   
    }

    public void SetWheel(double magnitude, double angle) {
        SetMagnitude(magnitude);
        if (magnitude > RobotMap.DriveSpeedDeadband) {
            SetAngle(angle);
        }
    }

    // Telemetry

    /**
     * Get current robot-centric angle heading of this wheel
     * @return Robot-centric wheel heading 0-360°
     */
    public double GetCurrentHeading() { 
        return HeadingEncoder.GetRelativeAngle(); 
    }

    /**
     * Get temperature of the motors
     * @return Array[2] of motor temperatures [Drive, Rotator]
     */
    public double[] GetMotorTemps() {
        double[] temps = new double[] { DriveMotor.getMotorTemperature(), RotatorMotor.getMotorTemperature() };
        return temps;
    }

    public double GetDriverTemp() {
        return DriveMotor.getMotorTemperature();
    }

    public double GetRotatorTemp() {
        return RotatorMotor.getMotorTemperature();
    }

    public short GetDriverStickyFaults() {
        return DriveMotor.getStickyFaults();
    }

    public short GetRotatorStickyFaults() {
        return RotatorMotor.getStickyFaults();
    }

    public double GetDriverSpeed() {
        return DriveMotor.get();
    }

    public Dictionary<String, Object> GetAllStats() {
        Dictionary<String, Object> stats = new Hashtable<String, Object>();

        stats.put("DTemp", GetDriverTemp());
        stats.put("RTemp", GetRotatorTemp());
        stats.put("DFaults", GetDriverStickyFaults());
        stats.put("RFaults", GetRotatorStickyFaults());
        stats.put("Speed", GetDriverSpeed());
        stats.put("Heading", GetCurrentHeading());

        return stats;
    }
}