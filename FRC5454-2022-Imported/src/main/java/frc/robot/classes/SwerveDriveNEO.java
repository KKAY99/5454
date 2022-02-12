/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.classes;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Constants;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;

/**
 * This class is used to run swerve drive.
 */
public class SwerveDriveNEO {

    private final double trackWidth = 20.625;
    private final double wheelBase = 24.375;
    private final double ticksPerRotation =42; // in encoder counts
    // KK Speed Adjustment 02/2021
    private final double kSpeedModifier = 0.8;
    private AHRS gyro;
    private double gyroAngle;

    private SparkMaxPIDController m_FrontLeftSteeringpid;
    private SparkMaxPIDController m_FrontRightSteeringpid;
    private SparkMaxPIDController m_BackLeftSteeringpid;
    private SparkMaxPIDController m_BackRightSteeringpid;
    private CANSparkMax m_FrontLeftSteering;
    private CANSparkMax m_FrontRightSteering;
    private CANSparkMax m_BackLeftSteering;
    private CANSparkMax m_BackRightSteering;

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

    // How to get offsets
    // Point all gears towards the inside of the robot, facing forwards.
    // Ex: Front left bevel gear faces front right bevel gear
    // Spin all wheels 1 rotation at a time until the value is between 0 and -1024
    // Check Actual encoder values on Swerve shuffleboard tab
    // Make sure wheels are straight with each other using a straight ...
    // ... edge like the yardstick

    public static final int frontLeftOffset = 0;
    public static final int frontRightOffset=0;

    public static final int backLeftOffset = 0;
    public static final int backRightOffset = 0;

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
    public double degreesToTicks = ticksPerRotation / 360.0;
    public double ticksToDegrees = 360.0 / ticksPerRotation;

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

    public SwerveDriveNEO(AHRS ahrs) {
        // Moved all of setup / init to the contained in the class
        // needs the gyro passed in
        // Create Drive Motors
         
        // #region Initialize Drive Motors
        m_FrontLeftSteering = new CANSparkMax(Constants.SwerveDriveNEO.kFrontLeftSteering, MotorType.kBrushless);
        m_FrontRightSteering = new CANSparkMax(Constants.SwerveDriveNEO.kFrontRightSteering, MotorType.kBrushless);
        m_BackLeftSteering = new CANSparkMax(Constants.SwerveDriveNEO.kBackLeftSteering, MotorType.kBrushless);
        m_BackRightSteering = new CANSparkMax(Constants.SwerveDriveNEO.kBackRightSteering, MotorType.kBrushless);
        m_FrontLeftDrive = new CANSparkMax(Constants.SwerveDriveNEO.kFrontLeftDrive, MotorType.kBrushless);
        m_FrontRightDrive = new CANSparkMax(Constants.SwerveDriveNEO.kFrontRightDrive, MotorType.kBrushless);
        m_BackLeftDrive = new CANSparkMax(Constants.SwerveDriveNEO.kBackLeftDrive, MotorType.kBrushless);
        m_BackRightDrive = new CANSparkMax(Constants.SwerveDriveNEO.kBackRightDrive, MotorType.kBrushless);
        // #endregion
        SwerveDriveInit(ahrs);
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

    private void SwerveDriveInit( AHRS ahrs) {

        gyro = ahrs;

        try {
            gyro = new AHRS(SPI.Port.kMXP);
            gyro.zeroYaw();
        } catch (Exception ex) {
            DriverStation.reportError("Error instantiating navX MXP:  " + ex.getMessage(), true);
        }
          
        initDrivingMotor(m_FrontLeftDrive);
        initDrivingMotor(m_BackLeftDrive);
        initDrivingMotor(m_BackLeftDrive);
        initDrivingMotor(m_BackRightDrive);
        m_FrontLeftSteeringpid=m_FrontLeftSteering.getPIDController();
        m_FrontRightSteeringpid=m_FrontRightSteering.getPIDController();
        m_BackRightSteeringpid=m_BackRightSteering.getPIDController();
        m_BackLeftSteeringpid=m_BackLeftSteering.getPIDController();
          
        
        initSteeringMotor(m_BackLeftSteering,m_BackLeftSteeringpid);
        initSteeringMotor(m_BackRightSteering,m_BackRightSteeringpid);
        initSteeringMotor(m_FrontRightSteering,m_FrontRightSteeringpid);
        initSteeringMotor(m_FrontLeftSteering,m_FrontLeftSteeringpid);
        
    }
    private void initDrivingMotor (CANSparkMax motor){
        motor.setIdleMode(IdleMode.kBrake);
        motor.setOpenLoopRampRate(0.5);
        motor.setSmartCurrentLimit(40);         
        motor.burnFlash();

    }
    private void initSteeringMotor(CANSparkMax motor,SparkMaxPIDController encoder)
    {   motor.restoreFactoryDefaults();
        motor.setInverted(Constants.SwerveDriveNEO.kMotorInvert);
        motor.setIdleMode(IdleMode.kBrake);
        encoder.setP(Constants.SwerveDriveNEO.kGains.kP);
        encoder.setI(Constants.SwerveDriveNEO.kGains.kI);
        encoder.setD(Constants.SwerveDriveNEO.kGains.kD);
        encoder.setIZone(Constants.SwerveDriveNEO.kGains.kIzone);
        encoder.setFF(Constants.SwerveDriveNEO.kGains.kF);
        encoder.setOutputRange(Constants.SwerveDriveNEO.minVel, Constants.SwerveDriveNEO.maxVel);
        encoder.setSmartMotionMaxAccel(Constants.SwerveDriveNEO.maxACC, Constants.SwerveDriveNEO.smartMotionSlot);
        encoder.setSmartMotionAllowedClosedLoopError(Constants.SwerveDriveNEO.allowedErr, Constants.SwerveDriveNEO.smartMotionSlot);
       
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

    private void driveRobot(double FWD, double STR, double RCW) {
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

        
        steer(m_FrontLeftSteering, frontLeft360Angle, frontLeftOffset);
        steer(m_FrontRightSteering, frontRight360Angle, frontRightOffset);
        steer(m_BackLeftSteering, backLeft360Angle, backLeftOffset);
        steer(m_BackRightSteering, backRight360Angle, backRightOffset);

        // frontLeftTargetPosition = -1 * ((ConvertAngleToPosition(frontLeft360Angle) +
        // Math.abs(frontLeftOffset)) % 1024);
        // frontRightTargetPosition = -1 * ((ConvertAngleToPosition(frontRight360Angle)
        // +
        // Math.abs(frontRightOffset)) % 1024);
        // backLeftTargetPosition = -1 * ((ConvertAngleToPosition(backLeft360Angle) +
        // Math.abs(backLeftOffset)) % 1024);
        // backRightTargetPosition = -1 * ((ConvertAngleToPosition(backRight360Angle) +
        // Math.abs(backRightOffset)) % 1024);

        // m_FrontRightSteering.set(ControlMode.Position, frontRightTargetPosition);
        // m_FrontLeftSteering.set(ControlMode.Position, frontLeftTargetPosition);
        // m_BackLeftSteering.set(ControlMode.Position, backLeftTargetPosition);
        // m_BackRightSteering.set(ControlMode.Position, backRightTargetPosition);

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
                if (Math.abs(gyroAngleError) > Constants.SwerveDriveNEO.kVisionGyroTolerance) {
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
     * if (Math.abs(xGap) > Constants.SwerveDriveNEO.kVisionXTolerance) {
     * if (xGap > 0) {
     * STR = Constants.SwerveDriveNEO.kVisionDriveSpeedSlow;
     * } else {
     * STR = -Constants.SwerveDriveNEO.kVisionDriveSpeedSlow;
     * }
     * } else if (Math.abs(distanceGap) >
     * Constants.SwerveDriveNEO.kVisionDistanceTolerance) {
     * if (distanceGap > 0) {
     * FWD = -Constants.SwerveDriveNEO.kVisionDriveSpeedSlow;
     * } else {
     * FWD = Constants.SwerveDriveNEO.kVisionDriveSpeedSlow;
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
     * driveSpeed = Constants.SwerveDriveNEO.kVisionDriveSpeedFast;
     * } else {
     * driveSpeed = Constants.SwerveDriveNEO.kVisionDriveSpeedSlow;
     * }
     * 
     * if (Math.abs(xGap) > Constants.SwerveDriveNEO.kVisionXTolerance) {
     * if (xGap > 0) {
     * STR = driveSpeed;
     * } else {
     * STR = -driveSpeed;
     * }
     * }
     * // hack instead of PID
     * if (Math.abs(distanceGap) > 3) {
     * driveSpeed = Constants.SwerveDriveNEO.kVisionDriveSpeedFast;
     * } else {
     * driveSpeed = Constants.SwerveDriveNEO.kVisionDriveSpeedSlow;
     * }
     * 
     * if (Math.abs(distanceGap) > Constants.SwerveDriveNEO.kVisionDistanceTolerance)
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
        if (Math.abs(xGap) > Constants.SwerveDriveNEO.kVisionXTolerance) {
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

        if (Math.abs(distanceGap) > Constants.SwerveDriveNEO.kVisionDistanceTolerance) {
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
     * quickSteer(m_FrontRightSteering, 0, false);
     * quickSteer(m_FrontLeftSteering, 0, false);
     * quickSteer(m_BackRightSteering, 0, false);
     * quickSteer(m_BackRightSteering, 0, false);
     * }
     */

    public void steer(CANSparkMax controller, double targetAngle, int offset) {
    
    
        //final double current = controller.getSelectedSensorPosition(Constants.SwerveDriveNEO.kSlotIdx);
        final double current = controller.getEncoder().getPosition();
        final double desired = (int) Math.round(targetAngle * ticksPerRotation / 360.0) + offset;

        final double newPosition = (int) MathUtil.minChange(desired, current, ticksPerRotation) + current;
        if (current != desired ) {
          System.out.println(controller.getDeviceId() + " - " +  current + " move to " + newPosition);
        }
          controller.getPIDController().setReference(newPosition, CANSparkMax.ControlType.kSmartMotion);
        
    }

    public double quickSteer(CANSparkMax controller, double targetAngle, boolean quickReverseAllowed) {
        double speedMultiplier = 1;
         //final double current = controller.getSelectedSensorPosition(Constants.SwerveDriveNEO.kSlotIdx);
        final double current = controller.getEncoder().getPosition();
        final double desired = (int) Math.round(targetAngle * ticksPerRotation / 360.0) - Math.abs(frontLeftOffset);

        if (quickReverseAllowed) {
            final double newPosition = (int) MathUtil.minChange(desired, current, ticksPerRotation / 2.0) + current;
            if (MathUtil.minDistance(newPosition, desired, ticksPerRotation) < .001) { // check if equal
                speedMultiplier = 1;
            } else {
                speedMultiplier = -1;
            }
            controller.getPIDController().setReference(newPosition, CANSparkMax.ControlType.kSmartMotion);
            return speedMultiplier;
        } else {
            speedMultiplier = 1;
            final double newPosition = (int) MathUtil.minChange(desired, current, ticksPerRotation) + current;
            controller.getPIDController().setReference(newPosition, CANSparkMax.ControlType.kSmartMotion);
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
        m_FrontRightSteering.getPIDController().setReference(frontLeftTargetPosition, CANSparkMax.ControlType.kSmartMotion);
        m_FrontLeftSteering.getPIDController().setReference(frontLeftTargetPosition, CANSparkMax.ControlType.kSmartMotion);
        m_BackLeftSteering.getPIDController().setReference(backLeftTargetPosition, CANSparkMax.ControlType.kSmartMotion);
        m_BackRightSteering.getPIDController().setReference(backRightTargetPosition, CANSparkMax.ControlType.kSmartMotion);

     /*   m_FrontRightSteering.set(ControlMode.Position, frontRightTargetPosition);
        m_FrontLeftSteering.set(ControlMode.Position, frontLeftTargetPosition);
        m_BackLeftSteering.set(ControlMode.Position, backLeftTargetPosition);
        m_BackRightSteering.set(ControlMode.Position, backRightTargetPosition);*/
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
        double TargetPosition = (ticksPerRotation / 360.0) * Angle;

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
        // while(Math.abs(xGap)>Constants.SwerveDriveNEO.kVisionXToleranceRCW){
        // RCW = m_Limelight.getRotationPower(xGap);
        // driveRobot(0,0,RCW);
        // xGap=m_Limelight.getX() + offset;
        // }
        do {
            xGap = m_Limelight.getX() + offset;
            RCW = m_Limelight.getRotationPower(xGap);
            driveRobot(0, 0, RCW);
        } while (Math.abs(xGap) > Constants.SwerveDriveNEO.kVisionXToleranceRCW);
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
    public void move(double direction, double speed, double distance, boolean stopAtEnd) {
        double xSpeedPercent;
        double ySpeedPercent;
        double startPosition;
        double targetPosition;
        double directionRadians;
        m_drivemode=true;
        directionRadians = Math.toRadians(direction);
        xSpeedPercent = Math.cos(directionRadians);
        ySpeedPercent = Math.sin(directionRadians);
        // Convert distance robot travels from inches
        distance = distance / ticksPerRotation;
        startPosition = m_FrontLeftDrive.getEncoder().getPosition();
        targetPosition = distance + startPosition;
        System.out.println("direction=" + direction + "(" + directionRadians + ")");
        System.out.println("targetdistance=" + targetPosition + "  currentdistance="
                + m_FrontLeftDrive.getEncoder().getPosition());
        System.out.println("xSpeedPer=" + xSpeedPercent + " ySpeedPer=" + ySpeedPercent);
        while (targetPosition > m_FrontLeftDrive.getEncoder().getPosition() && m_drivemode) {
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

}
