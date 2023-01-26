/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.classes.drives.oldswerve;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;

import frc.robot.classes.constants.*;
import frc.robot.classes.utilfuncs.*;

/**
 * This class is used to run swerve drive.
 */
public class SwerveDrive {

    private final double trackWidth = 20.625;
    private final double wheelBase = 24.375;

    private AHRS gyro;
    private double gyroAngle;

    // Create TalonSRX Speed Controllers
    private TalonSRX m_SrxFrontLeftSteering;
    private TalonSRX m_SrxFrontRightSteering;
    private TalonSRX m_SrxBackLeftSteering;
    private TalonSRX m_SrxBackRightSteering;

    // Create SpeedControllers
    private CANSparkMax m_FrontLeftDrive;
    private CANSparkMax m_FrontRightDrive;
    private CANSparkMax m_BackLeftDrive;
    private CANSparkMax m_BackRightDrive;

    // Create Variables
    public double frontRightAngle = 0;
    public double frontLeftAngle = 0;
    public double backLeftAngle = 0;
    public double backRightAngle = 0;

    public double frontRight360Angle = 0;
    public double frontLeft360Angle = 0;
    public double backLeft360Angle = 0;
    public double backRight360Angle = 0;

    public double frontLeftTargetPosition = 0;
    public double frontRightTargetPosition = 0;
    public double backLeftTargetPosition = 0;
    public double backRightTargetPosition = 0;

    public double frontLeftCurrentPosition = 0;
    public double frontRightCurrentPosition = 0;
    public double backLeftCurrentPosition = 0;
    public double backRightCurrentPosition = 0;

    public double frontRightSpeed = 0;
    public double frontLeftSpeed = 0;
    public double backLeftSpeed = 0;
    public double backRightSpeed = 0;

    //public int frontLeftOffset = -172;
    //public int frontRightOffset = -434;
    public int frontLeftOffset = -900;
    public int frontRightOffset = -900;
     public int backLeftOffset = -127;
    public int backRightOffset = -936;

    public double FWD = 0;
    public double STR = 0;
    public double RCW = 0;

    double R = Math.sqrt(Math.pow(wheelBase, 2) + Math.pow(trackWidth, 2));

    double A = 0.0;
    double B = 0.0;
    double C = 0.0;
    double D = 0.0;

    public final double defaultDeadzone = 0.07;
    public final double shootingRotationDeadzone = 0.01;
    public final double defaultRotationDeadzone = 0.2;

    public boolean isZero = false;

    public double degreesToRadians = Math.PI / 180.00;
    public double degreesToTicks = 1024.0 / 360.0;
    public double ticksToDegrees = 360.0 / 1024.0;

    /**
     * Configures the drivebase with Steering TalonSRXs and Speed Controller Drives
     * 
     * @param motorFrontLeftSteering  The front left steering TalonSRX
     * @param motorFrontRightSteering The front right steering TalonSRX
     * @param motorBackLeftSteering   The back left steering TalonSRX
     * @param motorBackRightSteering  The back right steering TalonSRX
     * 
     * @param motorFrontLeftDrive     The front left drive speed controller
     * @param motorFrontRightDrive    The front right drive speed controller
     * @param motorBackLeftDrive      The back left drive speed controller
     * @param motorBackRightDrive     The back right drive speed controller
     * 
     * @param defaultDeadzone         The default for Switchboard deadzone value
     */
    public SwerveDrive(TalonSRX motorFrontLeftSteering, TalonSRX motorFrontRightSteering,
            TalonSRX motorBackLeftSteering, TalonSRX motorBackRightSteering, CANSparkMax FrontLeftDrive,
            CANSparkMax FrontRightDrive, CANSparkMax BackLeftDrive, CANSparkMax BackRightDrive, AHRS ahrs) {

        gyro = ahrs;

        try {
            gyro = new AHRS(SPI.Port.kMXP);
            gyro.zeroYaw();
        } catch (Exception ex) {
            DriverStation.reportError("Error instantiating navX MXP:  " + ex.getMessage(), true);
        }

        m_SrxFrontLeftSteering = motorFrontLeftSteering;
        m_SrxFrontRightSteering = motorFrontRightSteering;
        m_SrxBackLeftSteering = motorBackLeftSteering;
        m_SrxBackRightSteering = motorBackRightSteering;

        m_FrontLeftDrive = FrontLeftDrive;
        m_FrontRightDrive = FrontRightDrive;
        m_BackLeftDrive = BackLeftDrive;
        m_BackRightDrive = BackRightDrive;

        m_FrontLeftDrive.setMotorType(MotorType.kBrushless);
        m_FrontRightDrive.setMotorType(MotorType.kBrushless);
        m_BackLeftDrive.setMotorType(MotorType.kBrushless);
        m_BackRightDrive.setMotorType(MotorType.kBrushless);

        m_FrontLeftDrive.setIdleMode(IdleMode.kCoast);
        m_FrontRightDrive.setIdleMode(IdleMode.kCoast);
        m_BackLeftDrive.setIdleMode(IdleMode.kCoast);
        m_BackRightDrive.setIdleMode(IdleMode.kCoast);

        m_FrontLeftDrive.setOpenLoopRampRate(0.5);
        m_FrontRightDrive.setOpenLoopRampRate(0.5);
        m_BackLeftDrive.setOpenLoopRampRate(0.5);
        m_BackRightDrive.setOpenLoopRampRate(0.5);

        m_FrontLeftDrive.setSmartCurrentLimit(40);
        m_FrontRightDrive.setSmartCurrentLimit(40);
        m_BackLeftDrive.setSmartCurrentLimit(40);
        m_BackRightDrive.setSmartCurrentLimit(40);

        m_FrontLeftDrive.burnFlash();
        m_FrontRightDrive.burnFlash();
        m_BackLeftDrive.burnFlash();
        m_BackRightDrive.burnFlash();

        // Configure Feedback Sensors for Drive
        m_SrxFrontLeftSteering.configSelectedFeedbackSensor(FeedbackDevice.Analog, Constants.kPIDLoopIdx,
                Constants.kTimeoutMs);
        m_SrxFrontRightSteering.configSelectedFeedbackSensor(FeedbackDevice.Analog, Constants.kPIDLoopIdx,
                Constants.kTimeoutMs);
        m_SrxBackLeftSteering.configSelectedFeedbackSensor(FeedbackDevice.Analog, Constants.kPIDLoopIdx,
                Constants.kTimeoutMs);
        m_SrxBackRightSteering.configSelectedFeedbackSensor(FeedbackDevice.Analog, Constants.kPIDLoopIdx,
                Constants.kTimeoutMs);

        // Tell the talon to not report if the sensor is out of Phase
        m_SrxFrontLeftSteering.setSensorPhase(Constants.kSensorPhase);
        m_SrxFrontRightSteering.setSensorPhase(Constants.kSensorPhase);
        m_SrxBackLeftSteering.setSensorPhase(Constants.kSensorPhase);
        m_SrxBackRightSteering.setSensorPhase(Constants.kSensorPhase);

        /*
         * Set based on what direction you want forward/positive to be. This does not
         * affect sensor phase.
         */
        m_SrxFrontLeftSteering.setInverted(Constants.kMotorInvert);
        m_SrxFrontRightSteering.setInverted(Constants.kMotorInvert);
        m_SrxBackLeftSteering.setInverted(Constants.kMotorInvert);
        m_SrxBackRightSteering.setInverted(Constants.kMotorInvert);

        /* Config the peak and nominal outputs, 12V means full */
        // Front Left
        m_SrxFrontLeftSteering.configNominalOutputForward(0, Constants.kTimeoutMs);
        m_SrxFrontLeftSteering.configNominalOutputReverse(0, Constants.kTimeoutMs);
        m_SrxFrontLeftSteering.configPeakOutputForward(1, Constants.kTimeoutMs);
        m_SrxFrontLeftSteering.configPeakOutputReverse(-1, Constants.kTimeoutMs);
        // Front Right
        m_SrxFrontRightSteering.configNominalOutputForward(0, Constants.kTimeoutMs);
        m_SrxFrontRightSteering.configNominalOutputReverse(0, Constants.kTimeoutMs);
        m_SrxFrontRightSteering.configPeakOutputForward(1, Constants.kTimeoutMs);
        m_SrxFrontRightSteering.configPeakOutputReverse(-1, Constants.kTimeoutMs);
        // Back Left
        m_SrxBackLeftSteering.configNominalOutputForward(0, Constants.kTimeoutMs);
        m_SrxBackLeftSteering.configNominalOutputReverse(0, Constants.kTimeoutMs);
        m_SrxBackLeftSteering.configPeakOutputForward(1, Constants.kTimeoutMs);
        m_SrxBackLeftSteering.configPeakOutputReverse(-1, Constants.kTimeoutMs);
        // Back Right
        m_SrxBackRightSteering.configNominalOutputForward(0, Constants.kTimeoutMs);
        m_SrxBackRightSteering.configNominalOutputReverse(0, Constants.kTimeoutMs);
        m_SrxBackRightSteering.configPeakOutputForward(1, Constants.kTimeoutMs);
        m_SrxBackRightSteering.configPeakOutputReverse(-1, Constants.kTimeoutMs);

        // Configure the amount of allowable error in the loop
        m_SrxFrontLeftSteering.configAllowableClosedloopError(Constants.kAlloweedError, Constants.kPIDLoopIdx,
                Constants.kTimeoutMs);
        m_SrxFrontRightSteering.configAllowableClosedloopError(Constants.kAlloweedError, Constants.kPIDLoopIdx,
                Constants.kTimeoutMs);
        m_SrxBackLeftSteering.configAllowableClosedloopError(Constants.kAlloweedError, Constants.kPIDLoopIdx,
                Constants.kTimeoutMs);
        m_SrxBackRightSteering.configAllowableClosedloopError(Constants.kAlloweedError, Constants.kPIDLoopIdx,
                Constants.kTimeoutMs);

        // Configure the overloop behavior of the encoders
        m_SrxFrontLeftSteering.configFeedbackNotContinuous(Constants.kNonContinuousFeedback, Constants.kTimeoutMs);
        m_SrxFrontRightSteering.configFeedbackNotContinuous(Constants.kNonContinuousFeedback, Constants.kTimeoutMs);
        m_SrxBackLeftSteering.configFeedbackNotContinuous(Constants.kNonContinuousFeedback, Constants.kTimeoutMs);
        m_SrxBackRightSteering.configFeedbackNotContinuous(Constants.kNonContinuousFeedback, Constants.kTimeoutMs);

        /* Config Position Closed Loop gains in slot0, typically kF stays zero. */
        // Front Left
        m_SrxFrontLeftSteering.config_kF(Constants.kPIDLoopIdx, Constants.kGains.kF, Constants.kTimeoutMs);
        m_SrxFrontLeftSteering.config_kP(Constants.kPIDLoopIdx, Constants.kGains.kP, Constants.kTimeoutMs);
        m_SrxFrontLeftSteering.config_kI(Constants.kPIDLoopIdx, Constants.kGains.kI, Constants.kTimeoutMs);
        m_SrxFrontLeftSteering.config_kD(Constants.kPIDLoopIdx, Constants.kGains.kD, Constants.kTimeoutMs);
        // Front Right
        m_SrxFrontRightSteering.config_kF(Constants.kPIDLoopIdx, Constants.kGains.kF, Constants.kTimeoutMs);
        m_SrxFrontRightSteering.config_kP(Constants.kPIDLoopIdx, Constants.kGains.kP, Constants.kTimeoutMs);
        m_SrxFrontRightSteering.config_kI(Constants.kPIDLoopIdx, Constants.kGains.kI, Constants.kTimeoutMs);
        m_SrxFrontRightSteering.config_kD(Constants.kPIDLoopIdx, Constants.kGains.kD, Constants.kTimeoutMs);
        // Back Left
        m_SrxBackLeftSteering.config_kF(Constants.kPIDLoopIdx, Constants.kGains.kF, Constants.kTimeoutMs);
        m_SrxBackLeftSteering.config_kP(Constants.kPIDLoopIdx, Constants.kGains.kP, Constants.kTimeoutMs);
        m_SrxBackLeftSteering.config_kI(Constants.kPIDLoopIdx, Constants.kGains.kI, Constants.kTimeoutMs);
        m_SrxBackLeftSteering.config_kD(Constants.kPIDLoopIdx, Constants.kGains.kD, Constants.kTimeoutMs);
        // Back Right
        m_SrxBackRightSteering.config_kF(Constants.kPIDLoopIdx, Constants.kGains.kF, Constants.kTimeoutMs);
        m_SrxBackRightSteering.config_kP(Constants.kPIDLoopIdx, Constants.kGains.kP, Constants.kTimeoutMs);
        m_SrxBackRightSteering.config_kI(Constants.kPIDLoopIdx, Constants.kGains.kI, Constants.kTimeoutMs);
        m_SrxBackRightSteering.config_kD(Constants.kPIDLoopIdx, Constants.kGains.kD, Constants.kTimeoutMs);
    }

    /**
     * Drives the Robot
     * 
     * FWD = Forward STR = Strafe Right RCW = Rotate Clockwise
     * 
     * Steering Angles: -180 to +180 measured clockwise with 0 being straight ahead
     * 
     * @param RCW_Joystick The left joystick X value.
     * @param FWD_Joystick The right joystick Y value.
     * @param STR_Joystick The right joystick X value.
     * @param fieldCentric Changes steering angles between robot and field centric
     * @param isShooting   Adjusts the rotation deadzone
     * @param gyroAngle    0 to 360 clockwise, 0 being straight down field
     * @param wheelBase    Measuement of wheelbase (Same units as trackwidth)
     * @param trackWidth   Measurement of trackwidth (Same units as wheelbase) Note:
     *                     Units for wheelbase and trackwidth don't matter so long
     *                     as they are the same, it is simply a ratio that is
     *                     calculated from them
     */
    public void drive(double RCW_Joystick, double FWD_Joystick, double STR_Joystick, boolean fieldCentric,
            boolean isShooting) {
        if (isShooting) {
            if (RCW_Joystick > shootingRotationDeadzone || RCW_Joystick < -shootingRotationDeadzone) {
                RCW = RCW_Joystick;
            } else {
                RCW = 0;
            }
        } else {
            if (RCW_Joystick > defaultRotationDeadzone || RCW_Joystick < -defaultRotationDeadzone) {
                RCW = RCW_Joystick;
            } else {
                RCW = 0;
            }
        }
        if (-FWD_Joystick > defaultDeadzone || -FWD_Joystick < -defaultDeadzone) {
            FWD = rescaleJoystickValue(defaultDeadzone, -FWD_Joystick);
        } else {
            FWD = 0;
        }
        if (STR_Joystick > defaultDeadzone || STR_Joystick < -defaultDeadzone) {
            STR = rescaleJoystickValue(defaultDeadzone, -STR_Joystick);
        } else {
            STR = 0;
        }

        // less bad
        if (fieldCentric) {
            gyroAngle = gyro.getFusedHeading();
            gyroAngle *= degreesToRadians;
            double temp = STR * Math.cos(gyroAngle) + FWD * Math.sin(gyroAngle);
            FWD = -STR * Math.sin(gyroAngle) + FWD * Math.cos(gyroAngle);
            STR = temp;
        }

        // copy of less bad
        // if (fieldCentric) {
        // gyroAngle = gyro.getFusedHeading();
        // gyroAngle *= degreesToRadians;
        // double temp = STR * Math.cos(gyroAngle) + FWD * Math.sin(gyroAngle);
        // FWD = -STR * Math.sin(gyroAngle) + FWD * Math.cos(gyroAngle);
        // STR = temp;
        // }

        // bad
        // if(fieldCentric)
        // {
        // gyroAngle = gyro.getFusedHeading();
        // gyroAngle *= degreesToRadians;
        // double temp = FWD * Math.cos(gyroAngle) + STR * Math.sin(gyroAngle);
        // STR = -FWD * Math.sin(gyroAngle) + STR * Math.cos(gyroAngle);
        // FWD = temp;
        // }

        // Math
        A = STR - RCW * (wheelBase / R);
        B = STR + RCW * (wheelBase / R);
        C = FWD - RCW * (trackWidth / R);
        D = FWD + RCW * (trackWidth / R);

        // Calculate speeds
        frontRightSpeed = Math.sqrt(Math.pow(B, 2) + Math.pow(C, 2));
        frontLeftSpeed = Math.sqrt(Math.pow(B, 2) + Math.pow(D, 2));
        backLeftSpeed = Math.sqrt(Math.pow(A, 2) + Math.pow(D, 2));
        backRightSpeed = Math.sqrt(Math.pow(A, 2) + Math.pow(C, 2));

        // Normalize speeds to between 0 and 1
        double max = frontRightSpeed;
        if (frontLeftSpeed > max) {
            max = frontLeftSpeed;
        }
        if (backLeftSpeed > max) {
            max = backLeftSpeed;
        }
        if (backRightSpeed > max) {
            max = backRightSpeed;
        }

        if (max > 1) {
            frontRightSpeed /= max;
            frontLeftSpeed /= max;
            backLeftSpeed /= max;
            backRightSpeed /= max;
        }

        backRightAngle = Math.atan2(B, C) * 180 / Math.PI;
        backLeftAngle = Math.atan2(B, D) * 180 / Math.PI;
        frontLeftAngle = Math.atan2(A, D) * 180 / Math.PI;
        frontRightAngle = Math.atan2(A, C) * 180 / Math.PI;

        frontRight360Angle = ConvertTo360Angle(frontRightAngle);
        frontLeft360Angle = ConvertTo360Angle(frontLeftAngle);
        backLeft360Angle = ConvertTo360Angle(backLeftAngle);
        backRight360Angle = ConvertTo360Angle(backRightAngle);

        m_FrontLeftDrive.set(frontLeftSpeed);
        m_FrontRightDrive.set(-frontRightSpeed);
        m_BackLeftDrive.set(backLeftSpeed);
        m_BackRightDrive.set(-backRightSpeed);

        steer(m_SrxFrontLeftSteering, frontLeft360Angle, frontLeftOffset);
        steer(m_SrxFrontRightSteering, frontRight360Angle, frontRightOffset);
        steer(m_SrxBackLeftSteering, backLeft360Angle, backLeftOffset);
        steer(m_SrxBackRightSteering, backRight360Angle, backRightOffset);

        // frontLeftTargetPosition = -1 * ((ConvertAngleToPosition(frontLeft360Angle) +
        // Math.abs(frontLeftOffset)) % 1024);
        // frontRightTargetPosition = -1 * ((ConvertAngleToPosition(frontRight360Angle)
        // +
        // Math.abs(frontRightOffset)) % 1024);
        // backLeftTargetPosition = -1 * ((ConvertAngleToPosition(backLeft360Angle) +
        // Math.abs(backLeftOffset)) % 1024);
        // backRightTargetPosition = -1 * ((ConvertAngleToPosition(backRight360Angle) +
        // Math.abs(backRightOffset)) % 1024);

        // m_SrxFrontRightSteering.set(ControlMode.Position, frontRightTargetPosition);
        // m_SrxFrontLeftSteering.set(ControlMode.Position, frontLeftTargetPosition);
        // m_SrxBackLeftSteering.set(ControlMode.Position, backLeftTargetPosition);
        // m_SrxBackRightSteering.set(ControlMode.Position, backRightTargetPosition);
    }

    public void steer(TalonSRX controller, double targetAngle, int offset) {
        final int ticksPerRotation = 1024; // in encoder counts
        final int current = (int) controller.getSelectedSensorPosition(Constants.kSlotIdx);
        
        final int desired = (int) Math.round(targetAngle * ticksPerRotation / 360.0) + offset;

        final int newPosition = (int) MathUtil.minChange(desired, current, ticksPerRotation) + current;
        controller.set(ControlMode.Position, newPosition);
    }

    public double quickSteer(TalonSRX controller, double targetAngle, boolean quickReverseAllowed) {
        double speedMultiplier = 1;
        final int ticksPerRotation = 1024; // in encoder counts
        final int current = (int) controller.getSelectedSensorPosition(Constants.kSlotIdx);
        final int desired = (int) Math.round(targetAngle * ticksPerRotation / 360.0) - Math.abs(frontLeftOffset);

        if (quickReverseAllowed) {
            final int newPosition = (int) MathUtil.minChange(desired, current, ticksPerRotation / 2.0) + current;
            if (MathUtil.minDistance(newPosition, desired, ticksPerRotation) < .001) { // check if equal
                speedMultiplier = 1;
            } else {
                speedMultiplier = -1;
            }
            controller.set(ControlMode.Position, newPosition);
            return speedMultiplier;
        } else {
            speedMultiplier = 1;
            final int newPosition = (int) MathUtil.minChange(desired, current, ticksPerRotation) + current;
            controller.set(ControlMode.Position, newPosition);
            return speedMultiplier;
        }
    }

    public void driveKill() {
        m_FrontLeftDrive.set(0);
        m_FrontRightDrive.set(0);
        m_BackLeftDrive.set(0);
        m_BackRightDrive.set(0);
    }

    public void driveZero() {
        m_SrxFrontRightSteering.set(ControlMode.Position, frontRightTargetPosition);
        m_SrxFrontLeftSteering.set(ControlMode.Position, frontLeftTargetPosition);
        m_SrxBackLeftSteering.set(ControlMode.Position, backLeftTargetPosition);
        m_SrxBackRightSteering.set(ControlMode.Position, backRightTargetPosition);
    }

    public double ConvertTo360Angle(double Angle) {
        // Convert from 0 -> -180/180 to 0 -> 360
        // Create a new output that is in the correct format
        double AdjustedAngle;

        if (Angle < 0) {
            AdjustedAngle = 360 + Angle;
        } else {
            AdjustedAngle = Angle;
        }

        return AdjustedAngle;
    }

    public double ConvertAngleToPosition(double Angle) {
        double TargetPosition = (1024.0 / 360.0) * Angle;

        return TargetPosition;
    }

    public double getSpeed() {
        return Math.hypot(FWD, STR) / Math.sqrt(2);
    }

    /**
     * Takes a minimum input and an input, and rescales it from -1 to 1
     * 
     * @param minimumInput The left joystick X value.
     * @param input        The right joystick Y value.
     * 
     */
    public static double rescaleJoystickValue(double minimumInput, double input) {
        double rMin = Math.abs(minimumInput); // the minimum possible input value
        double rMax = 1.0; // the maximum possible input value
        double tMin = 0.0; // the minimum possible scale value
        double tMax = 1.0; // the maximum possible scale value
        double scaledValue = 0.0;

        scaledValue = (tMax - tMin) * ((input - rMin) / (rMax - rMin)) + tMin;

        if (minimumInput < 1) {
            minimumInput *= -1;
        }

        return scaledValue;
    }

    /**
     * Converts your input angle 180 -> -180 to a 0 - 360 angle
     * 
     * @param inputAngle Value betweekn 180 -> -180
     * @return Value from 0 -> 360
     */
    public double ShuffleBoardAngleConversion(double inputAngle) {
        // Convert from 0 -> -180/180 to 0 -> 360
        // Create a new output that is in the correct format
        double ShuffleboardAdjustedAngle;
        if (inputAngle < 0) {
            ShuffleboardAdjustedAngle = 180 + (180 + inputAngle);
        } else {
            ShuffleboardAdjustedAngle = inputAngle;
        }

        return ShuffleboardAdjustedAngle;
    }

}
