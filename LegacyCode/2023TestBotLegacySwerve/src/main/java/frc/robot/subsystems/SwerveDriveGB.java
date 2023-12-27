/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;
import edu.wpi.first.wpilibj.Timer;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.classes.Limelight;
import frc.robot.classes.MathUtil;
import frc.robot.common.drivers.NavX;

//import jdk.jfr.events.ActiveRecordingEvent;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;

/**
 * This class is used to run swerve drive.
 */
public class SwerveDriveGB extends SubsystemBase{

    private final double trackWidth = 20.625;
    private final double wheelBase = 24.375;
    // KK Speed Adjustment 02/2021
    private final double kSpeedModifier = 0.8;
   // private AHRS gyro;
    private NavX gyro;
    private double gyroAngle;

    // Create TalonSRX Speed Controllers
    private TalonSRX m_SrxFrontLeftSteering;
    private TalonSRX m_SrxFrontRightSteering;
    private TalonSRX m_SrxBackLeftSteering;
    private TalonSRX m_SrxBackRightSteering;

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

    // How to get offsets
    // Point all gears towards the inside of the robot, facing forwards.
    // Ex: Front left bevel gear faces front right bevel gear
    // Spin all wheels 1 rotation at a time until the value is between 0 and -1024
    // Check Actual encoder values on Swerve shuffleboard tab
    // Make sure wheels are straight with each other using a straight ...
    // ... edge like the yardstick

    public static final int frontLeftOffset = -639;//3301;
    public static final int frontRightOffset = -406;//-764;

    public static final int backLeftOffset = 269;// -639;
    public static final int backRightOffset = -764;//-406;

    // Offset backup 5/19/21 3:04pm
    // public static final int frontLeftOffset = 513;
    // public static final int frontRightOffset = 272;
    // public static final int backLeftOffset = 396;
    // public static final int backRightOffset =-400;

    public double m_FWD = 0;
    public double m_STR = 0;
    public double m_RCW = 0;

    double R = Math.sqrt(Math.pow(wheelBase, 2) + Math.pow(trackWidth, 2));

    double A = 0.0;
    double B = 0.0;
    double C = 0.0;
    double D = 0.0;

    public final double defaultDeadzone = 0.09;
    public final double expandedDeadzone = 0.10;
    public final double shootingRotationDeadzone = 0.01;
    public final double defaultRotationDeadzone = 0.2;
    public boolean isZero = false;

    public double degreesToRadians = Math.PI / 180.00;
    public double degreesToTicks = 1024.0 / 360.0;
    public double ticksToDegrees = 360.0 / 1024.0;

    private boolean m_drivemode=false;
    // AutDrive by Limelight
    private boolean m_autoDrive = false;
    private boolean m_isShootingLong = false;
    private boolean m_isShootingShort = false;
    private Limelight m_Limelight;
    private double m_targetDistance;
    private double m_targetXPos;
    private boolean m_isAligned = false;

    private boolean gyroAlignedZero = false;
    private double gyroAngleError;

    public SwerveDriveGB(NavX navX) {
        // Moved all of setup / init to the contained in the class
        // needs the gyro passed in
        // Create Drive Motors
        
        TalonSRX FrontLeftSteering;
        TalonSRX FrontRightSteering;
        TalonSRX BackLeftSteering;
        TalonSRX BackRightSteering;
        CANSparkMax FrontLeftDrive;
        CANSparkMax FrontRightDrive;
        CANSparkMax BackLeftDrive;
        CANSparkMax BackRightDrive;
        // #region Initialize Drive Motors
        FrontLeftSteering = new TalonSRX(Constants.SwerveDriveGBConfig.kFrontLeftSteering);
        FrontRightSteering = new TalonSRX(Constants.SwerveDriveGBConfig.kFrontRightSteering);
        BackLeftSteering = new TalonSRX(Constants.SwerveDriveGBConfig.kBackLeftSteering);
        BackRightSteering = new TalonSRX(Constants.SwerveDriveGBConfig.kBackRightSteering);
        FrontLeftDrive = new CANSparkMax(Constants.SwerveDriveGBConfig.kFrontLeftDrive, MotorType.kBrushless);
        FrontRightDrive = new CANSparkMax(Constants.SwerveDriveGBConfig.kFrontRightDrive, MotorType.kBrushless);
        BackLeftDrive = new CANSparkMax(Constants.SwerveDriveGBConfig.kBackLeftDrive, MotorType.kBrushless);
        BackRightDrive = new CANSparkMax(Constants.SwerveDriveGBConfig.kBackRightDrive, MotorType.kBrushless);
        // #endregion
        SwerveDriveInit(FrontLeftSteering, FrontRightSteering, BackLeftSteering, BackRightSteering, FrontLeftDrive,
                FrontRightDrive, BackLeftDrive, BackRightDrive, navX);
    }

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

    private void SwerveDriveInit(TalonSRX motorFrontLeftSteering, TalonSRX motorFrontRightSteering,
            TalonSRX motorBackLeftSteering, TalonSRX motorBackRightSteering, CANSparkMax FrontLeftDrive,
            CANSparkMax FrontRightDrive, CANSparkMax BackLeftDrive, CANSparkMax BackRightDrive, NavX ahrs) {

        gyro = ahrs;

        

        m_SrxFrontLeftSteering = motorFrontLeftSteering;
        m_SrxFrontRightSteering = motorFrontRightSteering;
        m_SrxBackLeftSteering = motorBackLeftSteering;
        m_SrxBackRightSteering = motorBackRightSteering;

        m_FrontLeftDrive = FrontLeftDrive;
        m_FrontRightDrive = FrontRightDrive;
        m_BackLeftDrive = BackLeftDrive;
        m_BackRightDrive = BackRightDrive;

        m_FrontLeftDrive.setIdleMode(IdleMode.kBrake);
        m_FrontRightDrive.setIdleMode(IdleMode.kBrake);
        m_BackLeftDrive.setIdleMode(IdleMode.kBrake);
        m_BackRightDrive.setIdleMode(IdleMode.kBrake);

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
        m_SrxFrontLeftSteering.configSelectedFeedbackSensor(FeedbackDevice.Analog, Constants.SwerveDriveGBConfig.kPIDLoopIdx,
                Constants.SwerveDriveGBConfig.kTimeoutMs);
        m_SrxFrontRightSteering.configSelectedFeedbackSensor(FeedbackDevice.Analog, Constants.SwerveDriveGBConfig.kPIDLoopIdx,
                Constants.SwerveDriveGBConfig.kTimeoutMs);
        m_SrxBackLeftSteering.configSelectedFeedbackSensor(FeedbackDevice.Analog, Constants.SwerveDriveGBConfig.kPIDLoopIdx,
                Constants.SwerveDriveGBConfig.kTimeoutMs);
        m_SrxBackRightSteering.configSelectedFeedbackSensor(FeedbackDevice.Analog, Constants.SwerveDriveGBConfig.kPIDLoopIdx,
                Constants.SwerveDriveGBConfig.kTimeoutMs);

        // Tell the talon to not report if the sensor is out of Phase
        m_SrxFrontLeftSteering.setSensorPhase(Constants.SwerveDriveGBConfig.kSensorPhase);
        m_SrxFrontRightSteering.setSensorPhase(Constants.SwerveDriveGBConfig.kSensorPhase);
        m_SrxBackLeftSteering.setSensorPhase(Constants.SwerveDriveGBConfig.kSensorPhase);
        m_SrxBackRightSteering.setSensorPhase(Constants.SwerveDriveGBConfig.kSensorPhase);

        /*
         * Set based on what direction you want forward/positive to be. This does not
         * affect sensor phase.
         */
        m_SrxFrontLeftSteering.setInverted(Constants.SwerveDriveGBConfig.kMotorInvert);
        m_SrxFrontRightSteering.setInverted(Constants.SwerveDriveGBConfig.kMotorInvert);
        m_SrxBackLeftSteering.setInverted(Constants.SwerveDriveGBConfig.kMotorInvert);
        m_SrxBackRightSteering.setInverted(Constants.SwerveDriveGBConfig.kMotorInvert);

        /* Config the peak and nominal outputs, 12V means full */
        // Front Left
        m_SrxFrontLeftSteering.configNominalOutputForward(0, Constants.SwerveDriveGBConfig.kTimeoutMs);
        m_SrxFrontLeftSteering.configNominalOutputReverse(0, Constants.SwerveDriveGBConfig.kTimeoutMs);
        m_SrxFrontLeftSteering.configPeakOutputForward(1, Constants.SwerveDriveGBConfig.kTimeoutMs);
        m_SrxFrontLeftSteering.configPeakOutputReverse(-1, Constants.SwerveDriveGBConfig.kTimeoutMs);
        // Front Right
        m_SrxFrontRightSteering.configNominalOutputForward(0, Constants.SwerveDriveGBConfig.kTimeoutMs);
        m_SrxFrontRightSteering.configNominalOutputReverse(0, Constants.SwerveDriveGBConfig.kTimeoutMs);
        m_SrxFrontRightSteering.configPeakOutputForward(1, Constants.SwerveDriveGBConfig.kTimeoutMs);
        m_SrxFrontRightSteering.configPeakOutputReverse(-1, Constants.SwerveDriveGBConfig.kTimeoutMs);
        // Back Left
        m_SrxBackLeftSteering.configNominalOutputForward(0, Constants.SwerveDriveGBConfig.kTimeoutMs);
        m_SrxBackLeftSteering.configNominalOutputReverse(0, Constants.SwerveDriveGBConfig.kTimeoutMs);
        m_SrxBackLeftSteering.configPeakOutputForward(1, Constants.SwerveDriveGBConfig.kTimeoutMs);
        m_SrxBackLeftSteering.configPeakOutputReverse(-1, Constants.SwerveDriveGBConfig.kTimeoutMs);
        // Back Right
        m_SrxBackRightSteering.configNominalOutputForward(0, Constants.SwerveDriveGBConfig.kTimeoutMs);
        m_SrxBackRightSteering.configNominalOutputReverse(0, Constants.SwerveDriveGBConfig.kTimeoutMs);
        m_SrxBackRightSteering.configPeakOutputForward(1, Constants.SwerveDriveGBConfig.kTimeoutMs);
        m_SrxBackRightSteering.configPeakOutputReverse(-1, Constants.SwerveDriveGBConfig.kTimeoutMs);

        // Configure the amount of allowable error in the loop
        m_SrxFrontLeftSteering.configAllowableClosedloopError(Constants.SwerveDriveGBConfig.kAlloweedError,
                Constants.SwerveDriveGBConfig.kPIDLoopIdx,
                Constants.SwerveDriveGBConfig.kTimeoutMs);
        m_SrxFrontRightSteering.configAllowableClosedloopError(Constants.SwerveDriveGBConfig.kAlloweedError,
                Constants.SwerveDriveGBConfig.kPIDLoopIdx,
                Constants.SwerveDriveGBConfig.kTimeoutMs);
        m_SrxBackLeftSteering.configAllowableClosedloopError(Constants.SwerveDriveGBConfig.kAlloweedError,
                Constants.SwerveDriveGBConfig.kPIDLoopIdx,
                Constants.SwerveDriveGBConfig.kTimeoutMs);
        m_SrxBackRightSteering.configAllowableClosedloopError(Constants.SwerveDriveGBConfig.kAlloweedError,
                Constants.SwerveDriveGBConfig.kPIDLoopIdx,
                Constants.SwerveDriveGBConfig.kTimeoutMs);

        // Configure the overloop behavior of the encoders
        m_SrxFrontLeftSteering.configFeedbackNotContinuous(Constants.SwerveDriveGBConfig.kNonContinuousFeedback,
                Constants.SwerveDriveGBConfig.kTimeoutMs);
        m_SrxFrontRightSteering.configFeedbackNotContinuous(Constants.SwerveDriveGBConfig.kNonContinuousFeedback,
                Constants.SwerveDriveGBConfig.kTimeoutMs);
        m_SrxBackLeftSteering.configFeedbackNotContinuous(Constants.SwerveDriveGBConfig.kNonContinuousFeedback,
                Constants.SwerveDriveGBConfig.kTimeoutMs);
        m_SrxBackRightSteering.configFeedbackNotContinuous(Constants.SwerveDriveGBConfig.kNonContinuousFeedback,
                Constants.SwerveDriveGBConfig.kTimeoutMs);

        /* Config Position Closed Loop gains in slot0, typically kF stays zero. */
        // Front Left
        m_SrxFrontLeftSteering.config_kF(Constants.SwerveDriveGBConfig.kPIDLoopIdx, Constants.SwerveDriveGBConfig.kGains.kF,
                Constants.SwerveDriveGBConfig.kTimeoutMs);
        m_SrxFrontLeftSteering.config_kP(Constants.SwerveDriveGBConfig.kPIDLoopIdx, Constants.SwerveDriveGBConfig.kGains.kP,
                Constants.SwerveDriveGBConfig.kTimeoutMs);
        m_SrxFrontLeftSteering.config_kI(Constants.SwerveDriveGBConfig.kPIDLoopIdx, Constants.SwerveDriveGBConfig.kGains.kI,
                Constants.SwerveDriveGBConfig.kTimeoutMs);
        m_SrxFrontLeftSteering.config_kD(Constants.SwerveDriveGBConfig.kPIDLoopIdx, Constants.SwerveDriveGBConfig.kGains.kD,
                Constants.SwerveDriveGBConfig.kTimeoutMs);
        // Front Right
        m_SrxFrontRightSteering.config_kF(Constants.SwerveDriveGBConfig.kPIDLoopIdx, Constants.SwerveDriveGBConfig.kGains.kF,
                Constants.SwerveDriveGBConfig.kTimeoutMs);
        m_SrxFrontRightSteering.config_kP(Constants.SwerveDriveGBConfig.kPIDLoopIdx, Constants.SwerveDriveGBConfig.kGains.kP,
                Constants.SwerveDriveGBConfig.kTimeoutMs);
        m_SrxFrontRightSteering.config_kI(Constants.SwerveDriveGBConfig.kPIDLoopIdx, Constants.SwerveDriveGBConfig.kGains.kI,
                Constants.SwerveDriveGBConfig.kTimeoutMs);
        m_SrxFrontRightSteering.config_kD(Constants.SwerveDriveGBConfig.kPIDLoopIdx, Constants.SwerveDriveGBConfig.kGains.kD,
                Constants.SwerveDriveGBConfig.kTimeoutMs);
        // Back Left
        m_SrxBackLeftSteering.config_kF(Constants.SwerveDriveGBConfig.kPIDLoopIdx, Constants.SwerveDriveGBConfig.kGains.kF,
                Constants.SwerveDriveGBConfig.kTimeoutMs);
        m_SrxBackLeftSteering.config_kP(Constants.SwerveDriveGBConfig.kPIDLoopIdx, Constants.SwerveDriveGBConfig.kGains.kP,
                Constants.SwerveDriveGBConfig.kTimeoutMs);
        m_SrxBackLeftSteering.config_kI(Constants.SwerveDriveGBConfig.kPIDLoopIdx, Constants.SwerveDriveGBConfig.kGains.kI,
                Constants.SwerveDriveGBConfig.kTimeoutMs);
        m_SrxBackLeftSteering.config_kD(Constants.SwerveDriveGBConfig.kPIDLoopIdx, Constants.SwerveDriveGBConfig.kGains.kD,
                Constants.SwerveDriveGBConfig.kTimeoutMs);
        // Back Right
        m_SrxBackRightSteering.config_kF(Constants.SwerveDriveGBConfig.kPIDLoopIdx, Constants.SwerveDriveGBConfig.kGains.kF,
                Constants.SwerveDriveGBConfig.kTimeoutMs);
        m_SrxBackRightSteering.config_kP(Constants.SwerveDriveGBConfig.kPIDLoopIdx, Constants.SwerveDriveGBConfig.kGains.kP,
                Constants.SwerveDriveGBConfig.kTimeoutMs);
        m_SrxBackRightSteering.config_kI(Constants.SwerveDriveGBConfig.kPIDLoopIdx, Constants.SwerveDriveGBConfig.kGains.kI,
                Constants.SwerveDriveGBConfig.kTimeoutMs);
        m_SrxBackRightSteering.config_kD(Constants.SwerveDriveGBConfig.kPIDLoopIdx, Constants.SwerveDriveGBConfig.kGains.kD,
                Constants.SwerveDriveGBConfig.kTimeoutMs);

        m_SrxFrontLeftSteering.configSelectedFeedbackCoefficient(1.0);
        m_SrxFrontRightSteering.configSelectedFeedbackCoefficient(1.0);
        m_SrxBackLeftSteering.configSelectedFeedbackCoefficient(1.0);
        m_SrxBackRightSteering.configSelectedFeedbackCoefficient(1.0);

        m_SrxFrontLeftSteering.setNeutralMode(NeutralMode.Brake);
        m_SrxFrontRightSteering.setNeutralMode(NeutralMode.Brake);
        m_SrxBackLeftSteering.setNeutralMode(NeutralMode.Brake);
        m_SrxBackRightSteering.setNeutralMode(NeutralMode.Brake);
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

    public void driveRobot(double FWD, double STR, double RCW) {
        m_FWD = FWD;
        m_STR = STR;
        // Math
        A = STR - RCW * (wheelBase / R);
        B = STR + RCW * (wheelBase / R);
        C = FWD - RCW * (trackWidth / R);
        D = FWD + RCW * (trackWidth / R);
        // System.out.println(m_autoDrive + "-" + FWD + "--" + C + "--"+ trackWidth +
        // "--" + R);

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

        // KK Speed Adjustment 02/2021
        m_FrontLeftDrive.set(frontLeftSpeed * kSpeedModifier);
        m_FrontRightDrive.set(-frontRightSpeed * kSpeedModifier);
        m_BackLeftDrive.set(backLeftSpeed * kSpeedModifier);
        m_BackRightDrive.set(-backRightSpeed * kSpeedModifier);

        
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

    public void drive(double RCW_Joystick, double FWD_Joystick, double STR_Joystick, boolean fieldCentric) {
        double FWD = 0;
        double STR = 0;
        double RCW = 0;

        if (RCW_Joystick > defaultRotationDeadzone || RCW_Joystick < -defaultRotationDeadzone) {
            RCW = RCW_Joystick;
        } else {
            RCW = 0;
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

            //using ahrs
            // gyro.getFusedHeading();

            //gyroAngle *= degreesToRadians;
           
            gyroAngle = gyro.getAngle().toRadians();
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
        // KK autoDrive Disable if any joystick movement is detected outside the
        // deadzone
        if (Math.abs(FWD_Joystick) > expandedDeadzone || Math.abs(STR_Joystick) > expandedDeadzone
                || Math.abs(RCW_Joystick) > 2 * expandedDeadzone) {

            if (m_autoDrive) {
                System.out.println("Switching Drive Mode" + FWD_Joystick + "," + STR_Joystick);
            }
            m_autoDrive = false;
            m_isShootingLong = false;
            m_isShootingShort = false;
        }
        if (m_autoDrive == false) {
            // primary joystick driven drive mode
            driveRobot(FWD, STR, RCW);
        } else {
            // autodrive by Limelight
            if (gyroAlignedZero) {
                if (m_Limelight.isTargetAvailible()) {
                    double currentdistance = m_Limelight.getDistance();
                    double xPos = m_Limelight.getX();
                    System.out.println(m_targetDistance + " * " + currentdistance);
                    MoveToTargetNew(m_targetDistance - currentdistance, m_targetXPos - xPos);
                } else {
                    // If the target is not available stop autodrive
                    driveKill();
                    m_autoDrive = false;
                    m_isShootingShort = false;
                    m_isShootingLong = false;
                }
            } else {
                gyroAngleError = (gyroAngle) * 50.0 / 180.0;
                if (Math.abs(gyroAngleError) > Constants.SwerveDriveGBConfig.kVisionGyroTolerance) {
                    driveRobot(0, 0, m_Limelight.getRotationPower(gyroAngleError));
                } else {
                    gyroAlignedZero = true;
                }

            }

        }

    }

    /*
     * private void MoveToTargetBasic(double distanceGap, double xGap) {
     * double FWD = 0;
     * double STR = 0;
     * double RCW = 0;
     * 
     * if (Math.abs(xGap) > Constants.SwerveDriveGBConfig.kVisionXTolerance) {
     * if (xGap > 0) {
     * STR = Constants.SwerveDriveGBConfig.kVisionDriveSpeedSlow;
     * } else {
     * STR = -Constants.SwerveDriveGBConfig.kVisionDriveSpeedSlow;
     * }
     * } else if (Math.abs(distanceGap) >
     * Constants.SwerveDriveGBConfig.kVisionDistanceTolerance) {
     * if (distanceGap > 0) {
     * FWD = -Constants.SwerveDriveGBConfig.kVisionDriveSpeedSlow;
     * } else {
     * FWD = Constants.SwerveDriveGBConfig.kVisionDriveSpeedSlow;
     * }
     * 
     * }
     * System.out.println("F is " + FWD + " STR is " + STR + " RCW is " + RCW);
     * driveRobot(FWD, STR, RCW);
     * 
     * }
     * 
     * private void MoveToTargetAdvanced(double distanceGap, double xGap) {
     * double FWD = 0;
     * double STR = 0;
     * double RCW = 0;
     * double driveSpeed;
     * // hack instead of PID
     * if (Math.abs(xGap) > 3) {
     * driveSpeed = Constants.SwerveDriveGBConfig.kVisionDriveSpeedFast;
     * } else {
     * driveSpeed = Constants.SwerveDriveGBConfig.kVisionDriveSpeedSlow;
     * }
     * 
     * if (Math.abs(xGap) > Constants.SwerveDriveGBConfig.kVisionXTolerance) {
     * if (xGap > 0) {
     * STR = driveSpeed;
     * } else {
     * STR = -driveSpeed;
     * }
     * }
     * // hack instead of PID
     * if (Math.abs(distanceGap) > 3) {
     * driveSpeed = Constants.SwerveDriveGBConfig.kVisionDriveSpeedFast;
     * } else {
     * driveSpeed = Constants.SwerveDriveGBConfig.kVisionDriveSpeedSlow;
     * }
     * 
     * if (Math.abs(distanceGap) > Constants.SwerveDriveGBConfig.kVisionDistanceTolerance)
     * {
     * if (distanceGap > 0) {
     * FWD = -driveSpeed;
     * } else {
     * FWD = driveSpeed;
     * }
     * 
     * }
     * if (!(FWD + STR + RCW == 0.0)) {
     * System.out.println("F is " + FWD + " STR is " + STR + " RCW is " + RCW);
     * }
     * driveRobot(FWD, STR, RCW);
     * // steer0();
     * 
     * }
     */

    private void MoveToTargetNew(double distanceGap, double xGap) { // ts 7/24/21: add intake during movement?
        double FWD = 0;
        double STR = 0;
        double RCW = 0;
        double driveSpeedSTR;
        double driveSpeedFWD;
        // kind of PID
        driveSpeedSTR = Math.abs(xGap) * 0.025;
        if ((driveSpeedSTR < 0.05) && (driveSpeedSTR > 0.0)) {
            driveSpeedSTR += 0.05;
        }
        if (Math.abs(xGap) > Constants.SwerveDriveGBConfig.kVisionXTolerance) {
            if (xGap > 0) {
                STR = driveSpeedSTR;
            } else {
                STR = -driveSpeedSTR;
            }
        }

        driveSpeedFWD = Math.abs(distanceGap) * 0.05;
        if ((driveSpeedFWD < 0.05) && (driveSpeedFWD > 0.0)) {
            driveSpeedFWD += 0.05;
        }

        if (Math.abs(distanceGap) > Constants.SwerveDriveGBConfig.kVisionDistanceTolerance) {
            if (distanceGap > 0) {
                FWD = -driveSpeedFWD;
            } else {
                FWD = driveSpeedFWD;
            }
        }
        if ((FWD + STR + RCW == 0.0)) {
            System.out.println("F is " + FWD + " STR is " + STR + " RCW is " + RCW);
            m_isAligned = true;
            // KK 11/1/21
            driveKill();
            m_autoDrive = false;

        } else {
            if (m_isAligned) {
                System.out.println("xx - F is " + FWD + " STR is " + STR + " RCW is " + RCW);
            }
            m_isAligned = false;

        }

        driveRobot(FWD, STR, RCW);
        // steer0();
    }

    /*
     * private void steer0() {
     * quickSteer(m_SrxFrontRightSteering, 0, false);
     * quickSteer(m_SrxFrontLeftSteering, 0, false);
     * quickSteer(m_SrxBackRightSteering, 0, false);
     * quickSteer(m_SrxBackRightSteering, 0, false);
     * }
     */

    public void steer(TalonSRX controller, double targetAngle, int offset) {
    
        final double ticksPerRotation = 1024; // in encoder counts
        final double current = controller.getSelectedSensorPosition(Constants.SwerveDriveGBConfig.kSlotIdx);
        final double desired = (int) Math.round(targetAngle * ticksPerRotation / 360.0) + offset;

        final double newPosition = (int) MathUtil.minChange(desired, current, ticksPerRotation) + current;
        controller.set(ControlMode.Position, newPosition);
    }

    public double quickSteer(TalonSRX controller, double targetAngle, boolean quickReverseAllowed) {
        double speedMultiplier = 1;
        final int ticksPerRotation = 1024; // in encoder counts
        final double current = controller.getSelectedSensorPosition(Constants.SwerveDriveGBConfig.kSlotIdx);
        final double desired = (int) Math.round(targetAngle * ticksPerRotation / 360.0) - Math.abs(frontLeftOffset);

        if (quickReverseAllowed) {
            final double newPosition = (int) MathUtil.minChange(desired, current, ticksPerRotation / 2.0) + current;
            if (MathUtil.minDistance(newPosition, desired, ticksPerRotation) < .001) { // check if equal
                speedMultiplier = 1;
            } else {
                speedMultiplier = -1;
            }
            controller.set(ControlMode.Position, newPosition);
            return speedMultiplier;
        } else {
            speedMultiplier = 1;
            final double newPosition = (int) MathUtil.minChange(desired, current, ticksPerRotation) + current;
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
        return Math.hypot(m_FWD, m_STR) / Math.sqrt(2);
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

    public void targetAlign(Limelight activeLimelight, double targetDistance, double targetXPos, boolean isShootingLong,
            boolean isShootingShort) {
        m_targetDistance = targetDistance;
        m_targetXPos = targetXPos;
        m_Limelight = activeLimelight;
        gyroAlignedZero = false;
        if (activeLimelight.isTargetAvailible() == true) {
            m_isAligned = false;
            m_autoDrive = true;
            m_isShootingLong = isShootingLong;
            m_isShootingShort = isShootingShort;
        }

    }

    public void targetTurn(Limelight activeLimelight) {
        m_Limelight = activeLimelight;
        double RCW;
        double xGap;
        double offset = 6;
        m_autoDrive = true;
        // xGap=m_Limelight.getX() + offset;
        // xGap += 15;
        // while(Math.abs(xGap)>Constants.SwerveDriveGBConfig.kVisionXToleranceRCW){
        // RCW = m_Limelight.getRotationPower(xGap);
        // driveRobot(0,0,RCW);
        // xGap=m_Limelight.getX() + offset;
        // }
        do {
            xGap = m_Limelight.getX() + offset;
            RCW = m_Limelight.getRotationPower(xGap);
            driveRobot(0, 0, RCW);
        } while (Math.abs(xGap) > Constants.SwerveDriveGBConfig.kVisionXToleranceRCW);
        driveKill();
        m_autoDrive = false;
    }

    public void SetDriverMode() {
        m_autoDrive = false;
    }

    /**
     * For Autonomouse code this moves the boot the specified direction, speed, and
     * distnace
     */
    public void move(double direction, double rotation, double speed, double distance, double failTime, boolean stopAtEnd) {
        double xSpeedPercent;
        double ySpeedPercent;
        double startPosition;
        double targetPosition;
        double directionRadians;
        double startTime=Timer.getFPGATimestamp();
        double currentTime=Timer.getFPGATimestamp();
        double endTime;
        m_drivemode=true;
        directionRadians = Math.toRadians(direction);
        xSpeedPercent = Math.cos(directionRadians);
        ySpeedPercent = Math.sin(directionRadians);
        // Convert distance robot travels from inches
        distance = distance / 42;
        startPosition = m_BackLeftDrive.getEncoder().getPosition();
        targetPosition = distance + startPosition;
        System.out.println("direction=" + direction + "(" + directionRadians + ")");
        System.out.println("targetdistance=" + targetPosition + "  currentdistance="
                + m_BackLeftDrive.getEncoder().getPosition());
        System.out.println("xSpeedPer=" + xSpeedPercent + " ySpeedPer=" + ySpeedPercent);
        endTime=currentTime+failTime;
        while ((targetPosition > Math.abs(m_BackLeftDrive.getEncoder().getPosition())) && m_drivemode && currentTime<endTime) {
            System.out.println("targetPos" + targetPosition + " --CurrentPosition" + m_BackLeftDrive.getEncoder().getPosition());
             currentTime=Timer.getFPGATimestamp();
            drive((double) 0, xSpeedPercent * speed, ySpeedPercent * speed, false);
        }
        if (stopAtEnd) {

            driveKill();

        }
        m_drivemode=false;

    }
    public void resetDriveModes(){
        m_autoDrive=false;
        m_drivemode=false;
    }
    public boolean isAutoShootLong() {
        return m_isShootingLong;
    }

    public boolean isAutoShootShort() {
        return m_isShootingShort;
    }

    public boolean isReadyToShoot() {
        return m_isAligned && m_autoDrive;
    }
    public double getFrontLeftAngle(){
        double angle= m_SrxFrontLeftSteering.getSelectedSensorPosition(Constants.SwerveDriveGBConfig.kSlotIdx);
   
        return angle;
    }
    public double getFrontRightAngle(){
        double angle= m_SrxFrontRightSteering.getSelectedSensorPosition(Constants.SwerveDriveGBConfig.kSlotIdx);
   
        return angle;
    }
    public double getBackLeftAngle(){
        double angle= m_SrxBackLeftSteering.getSelectedSensorPosition(Constants.SwerveDriveGBConfig.kSlotIdx);
        System.out.println("Back Left " + angle) ;
        return angle;
    }
    public double getbackRightAngle(){
        double angle= m_SrxBackRightSteering.getSelectedSensorPosition(Constants.SwerveDriveGBConfig.kSlotIdx);
   
        return angle;
        }
    public void resetGyroscope() {
        gyro.setAdjustmentAngle(gyro.getUnadjustedAngle());
    }


}
