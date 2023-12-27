/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.Map;

import javax.swing.table.TableStringConverter;

import com.kauailabs.navx.frc.AHRS;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

import frc.robot.classes.constants.*;
import frc.robot.classes.constants.Constants.MotorControllers;
import frc.robot.classes.constants.Constants.SwerveModules;
import frc.robot.classes.mechanisms.*;
import frc.robot.classes.sensors.*;
import frc.robot.classes.drives.oldswerve.SwerveGyroAdapter;
import frc.robot.classes.drives.swervewpi.*;
import frc.robot.classes.utilfuncs.*;
import frc.robot.classes.LEDStrip;

public class Robot extends TimedRobot {

        // Create Joysticks
        Joystick m_joystickLeft;
        Joystick m_joystickRight;
        Joystick m_gamepad;

// KK 1/23/21
//        SwerveModule m_FrontLeftModule;
//       SwerveModule m_FrontRightModule;
//        SwerveModule m_BackLeftModule;
//        SwerveModule m_BackRightModule;

        // Create Intake Motors and Solenoids
        VictorSPX m_IntakeMotor;
        Solenoid m_IntakePnematic;
        Solenoid m_IntakeDrop;

        // Create Conveyor Controller
        TalonSRX m_ConveyorMotorTop;
        TalonSRX m_ConveyorMotorBottom;

        // Create Shooting motors
        CANSparkMax m_TopShooterMotor;
        CANSparkMax m_BottomShooterMotor;

        // Create Climb Motors
        TalonSRX m_ClimbPullMotor;
        VictorSPX m_ClimbLiftMotor;

        // Create the Gyro
        AHRS ahrs;

        // Create Shooting solenoids
        Solenoid m_LongShooterSolenoid;
        Solenoid m_ShortShooterSolenoid;
        Solenoid m_ShooterLights;

        // Create Custom Classes
        WpiSwerveDrive m_Drive = new WpiSwerveDrive();
        Intake m_Intake;
        Conveyor m_Conveyor;
        Shooter m_Shooter;
        Climber m_Climber;
        LEDStrip m_ledStrip;
        DistanceSensor m_DistanceSensor;

        AnalogInput conveyorSensor = new AnalogInput(1);
        AnalogInput intakeSonarSensor = new AnalogInput(0);
        AnalogInput conveyorShutdownSensor = new AnalogInput(2);
        AnalogInput shooterSonarSensor = new AnalogInput(3);

        // Create Variables
        boolean fieldCentric = false;
        boolean isShootingLong = false;
        boolean isShootingShort = false;
        double gyroAngle = 0.0;
        boolean shuffleboardRecording = false;

        Timer shooterPneumaticTimer;
        Timer shooterPneumaticResetTimer;
        boolean shooterPneumaticTimerSet;
        boolean shooterPneumaticRestTimerSet;

        final int LED_PWM_PORT = 9;
        int LED_Color = 1;
        int LED_START = 3;
        int NUM_LEDS = 135 + LED_START;

        double climbPullPower = 0.0;
        double climbRaisePower = 0.0;
        boolean isClimbing = true;
        double climbDeadzone = 0.1;

        boolean isIntakeDown = false;

        boolean conveyorIsLoaded = false;
        boolean conveyorSensorTriggered = false;
        boolean conveyorShutdownTriggered = true;
        Timer conveyorTimer;
        Timer conveyorDelayTimer;
        double conveyorLoadTime = 1.0;
        double conveyorFailedTime = 1.0;
        double conveyorShutoffDelay = 0.25;
        boolean conveyorTimerHasBeenSet = false;
        boolean conveyorDelayTimerIsSet = false;
        boolean conveyorButtonHasBeenPressed = false;

        BinarySwitch autoChooserSwitch;
        boolean autoTimerHasBeenSet = false;
        Timer autoTimer;
        double time;

        int currentState = AutoEnumeration.Delay;

        boolean shouldRun = true;
        boolean autoSubTimerHasBeenSet = false;
        boolean shouldShoot = true;
        boolean shouldDrive = true;
        boolean shouldDriveForward = true;
        boolean shouldMoveFirstPartner = true;
        boolean shouldMoveSecondPartner = true;

        boolean shouldRunShooter = false;

        double shooterSpinupTime = 1.5;
        double shooterLoadTime = 3.5;
        double shooterLoadSpeed = 0.45;
        double driveForwardTime = 0.75;
        double pushBackTime = 0.75;
        double drivePushPower = -0.5;
        double driveResetTime = 1.0;
        double driveResetPower = 0.2;
        double moveForwardTime = 1.0;
        double driveStrafeTime = 0.75;
        double strafePower = 0.5;
        double secondPushBackTime = 1.0;
        double secondPullForwardtime = 0.5;

        Timer autoSubTimer;
        double subTime;

        Timer delayTimer;
        double delay = 0.0;
        double maxDelay = 0.0;
        boolean delayTimerHasBeenSet;
        BinarySwitch delaySwitch;

        boolean m_gamepadY = false;
        boolean m_gamepadX = false;
        boolean m_gamepadA = false;
        boolean m_gamepadB = false;
        boolean m_gamepadRightBumper = false;
        boolean m_gamepadLeftBumper = false;
        double m_gamepadLeftTrigger = 0.0;
        double m_gamepadRightTrigger = 0.0;
        double m_gamepadRightJoystickY = 0.0;
        double m_gamepadLeftJoystickY = 0.0;

        int gamepadPOV = 0;

        boolean ShortUp = true;
        boolean ShortDown = false;
        boolean LongUp = false;
        boolean LongDown = true;
        boolean on = true;
        boolean off = false;

        // #region Shuffleboard
        // #region Create Shuffleboard Widgets for Angles
        private static SwerveGyroAdapter frontLeftGyroAdapter = new SwerveGyroAdapter();
        private static SwerveGyroAdapter frontRightGyroAdapter = new SwerveGyroAdapter();
        private static SwerveGyroAdapter backLeftGyroAdapter = new SwerveGyroAdapter();
        private static SwerveGyroAdapter backRightGyroAdapter = new SwerveGyroAdapter();
        // #endregion

        // #region Create Shuffleboard Tabs
        private static ShuffleboardTab SwerveTab = Shuffleboard.getTab("Swerve");
        private static ShuffleboardTab Joysticks = Shuffleboard.getTab("Joysticks");
        private static ShuffleboardTab SwerveEncoders = Shuffleboard.getTab("SwerveEncoders");
        private static ShuffleboardTab AutoTab = Shuffleboard.getTab("Auto");
        private static ShuffleboardTab SubSystems = Shuffleboard.getTab("SubSystems");
        // #endregion

        // #region NetworkEntries
        // Create Network Table Entries

        static NetworkTableEntry networkTableEntryShooterSonar = SubSystems.add("Shooter Sonar", 0)
                        .withWidget(BuiltInWidgets.kNumberBar).withSize(2, 2).getEntry();

        static NetworkTableEntry networkTableEntryShooterPower = AutoTab.add("Shooter Power", 0)
                        .withWidget(BuiltInWidgets.kNumberBar).withSize(2, 2).getEntry();

        static NetworkTableEntry networkTableEntryVisionDistance = AutoTab.add("Vision Distance", 0)
                        .withWidget(BuiltInWidgets.kNumberBar).withSize(2, 2).getEntry();

        static NetworkTableEntry networkTableEntryConveyorSensor = SubSystems.add("Conveyor Sensor Value", 0.0)
                        .withWidget(BuiltInWidgets.kNumberBar).withSize(2, 2).getEntry();

        static NetworkTableEntry networkTableEntryConveyorShuttoffSensor = SubSystems
                        .add("Conveyor Shutdown Value", 0.0).withWidget(BuiltInWidgets.kNumberBar).withSize(2, 2)
                        .getEntry();

        static NetworkTableEntry networkTableEntryConveyorTriggered = SubSystems.add("Conveyor Triggered", false)
                        .withWidget(BuiltInWidgets.kBooleanBox).withSize(2, 2).getEntry();

        static NetworkTableEntry networkTableEntryConveyorShutoffTriggered = SubSystems.add("Shutdown Triggered", false)
                        .withWidget(BuiltInWidgets.kBooleanBox).withSize(2, 2).getEntry();

        static NetworkTableEntry networkTableEntryJoystickX = Joysticks.add("Joystick X", 0)
                        .withWidget(BuiltInWidgets.kNumberBar).getEntry();
        static NetworkTableEntry networkTableEntryJoystickY = Joysticks.add("Joystick Y", 0)
                        .withWidget(BuiltInWidgets.kNumberBar).getEntry();
        static NetworkTableEntry networkTableEntryJoystickZ = Joysticks.add("Joystick Z", 0)
                        .withWidget(BuiltInWidgets.kNumberBar).getEntry();

        static NetworkTableEntry networkTableEntryFWD = Joysticks.add("FWD", 0).withWidget(BuiltInWidgets.kNumberBar)
                        .getEntry();
        static NetworkTableEntry networkTableEntryRCW = Joysticks.add("RCW", 0).withWidget(BuiltInWidgets.kNumberBar)
                        .getEntry();
        static NetworkTableEntry networkTableEntrySTR = Joysticks.add("STR", 0).withWidget(BuiltInWidgets.kNumberBar)
                        .getEntry();

        static NetworkTableEntry networkTableEntryFrontLeftSpeed = SwerveTab.add("FL Speed", 0)
                        .withWidget(BuiltInWidgets.kVoltageView)
                        .withProperties(Map.of("Min", 0, "Max", 1, "Center", 0, "Orientation", "VERTICAL"))
                        .withPosition(1, 0).withSize(2, 5).getEntry();

        static NetworkTableEntry networkTableEntryFrontRightSpeed = SwerveTab.add("FR Speed", 0)
                        .withWidget(BuiltInWidgets.kVoltageView)
                        .withProperties(Map.of("Min", 0, "Max", 1, "Center", 0, "Orientation", "VERTICAL"))
                        .withPosition(14, 0).withSize(2, 5).getEntry();

        static NetworkTableEntry networkTableEntryBackLeftSpeed = SwerveTab.add("BL Speed", 0)
                        .withWidget(BuiltInWidgets.kVoltageView)
                        .withProperties(Map.of("Min", 0, "Max", 1, "Center", 0, "Orientation", "VERTICAL"))
                        .withPosition(1, 5).withSize(2, 5).getEntry();

        static NetworkTableEntry networkTableEntryBackRightSpeed = SwerveTab.add("BR Speed", 0)
                        .withWidget(BuiltInWidgets.kVoltageView)
                        .withProperties(Map.of("Min", 0, "Max", 1, "Center", 0, "Orientation", "VERTICAL"))
                        .withPosition(14, 5).withSize(2, 5).getEntry();

        static NetworkTableEntry networkTableEntryFrontLeftEncoderActual = SwerveEncoders.add("FL Encoder Actual", 0)
                        .withWidget(BuiltInWidgets.kTextView).withPosition(0, 0).withSize(2, 1).getEntry();

        static NetworkTableEntry networkTableEntryFrontRightEncoderActual = SwerveEncoders.add("FR Encoder Actual", 0)
                        .withWidget(BuiltInWidgets.kTextView).withPosition(2, 0).withSize(2, 1).getEntry();

        static NetworkTableEntry networkTableEntryBackLeftEncoderActual = SwerveEncoders.add("BL Encoder Actual", 0)
                        .withWidget(BuiltInWidgets.kTextView).withPosition(0, 1).withSize(2, 1).getEntry();

        static NetworkTableEntry networkTableEntryBackRightEncoderActual = SwerveEncoders.add("BR Encoder Actual", 0)
                        .withWidget(BuiltInWidgets.kTextView).withPosition(2, 1).withSize(2, 1).getEntry();

        static NetworkTableEntry networkTableEntryFrontLeftEncoderTarget = SwerveEncoders.add("FL Encoder Target", 0)
                        .withWidget(BuiltInWidgets.kTextView).withPosition(5, 0).withSize(2, 1).getEntry();

        static NetworkTableEntry networkTableEntryFrontRightEncoderTarget = SwerveEncoders.add("FR Encoder Target", 0)
                        .withWidget(BuiltInWidgets.kTextView).withPosition(7, 0).withSize(2, 1).getEntry();

        static NetworkTableEntry networkTableEntryBackLeftEncoderTarget = SwerveEncoders.add("BL Encoder Target", 0)
                        .withWidget(BuiltInWidgets.kTextView).withPosition(5, 1).withSize(2, 1).getEntry();

        static NetworkTableEntry networkTableEntryBackRightEncoderTarget = SwerveEncoders.add("BR Encoder Target", 0)
                        .withWidget(BuiltInWidgets.kTextView).withPosition(7, 1).withSize(2, 1).getEntry();

        
        static NetworkTableEntry frontLeftAngle = SwerveEncoders.add("FL Angle", 0).withWidget(BuiltInWidgets.kTextView)
                        .withPosition(0, 3).withSize(2, 1).getEntry();
        static NetworkTableEntry frontRightAngle = SwerveEncoders.add("FR Angle", 0)
                        .withWidget(BuiltInWidgets.kTextView).withPosition(2, 3).withSize(2, 1).getEntry();
        static NetworkTableEntry backLeftAngle = SwerveEncoders.add("BL Angle", 0).withWidget(BuiltInWidgets.kTextView)
                        .withPosition(0, 4).withSize(2, 1).getEntry();
        static NetworkTableEntry backRightAngle = SwerveEncoders.add("BR Angle", 0).withWidget(BuiltInWidgets.kTextView)
                        .withPosition(2, 4).withSize(2, 1).getEntry();
        static NetworkTableEntry frontDriveDistance=SwerveEncoders.add("F Distance",0).withWidget(BuiltInWidgets.kTextView)
                        .withPosition(3, 4).withSize(2, 1).getEntry();

        static NetworkTableEntry ShuffleboardLog = SwerveEncoders.add("ShuffleboardLog", "")
                        .withWidget(BuiltInWidgets.kTextView).withSize(4, 2).withPosition(0, 6).getEntry();

        static NetworkTableEntry shuffleboardGyroFused = SubSystems.add("Gyro - Fused Heading", 0)
                        .withWidget(BuiltInWidgets.kTextView).getEntry();

        static String ShuffleboardLogString;
        // #endregion
        // #endregion

        /**
         * This function is run when the robot is first started up and should be used
         * for any initialization code.
         */
        @Override
        public void robotInit() {

                // #region Initialize Joysticks
                m_joystickLeft = new Joystick(0);
                m_joystickRight = new Joystick(1);
                m_gamepad = new Joystick(2);
                // #endregion
             // now being delcared in WPI Swerve Drive Class  KK 1/23/21             
             //   m_FrontLeftModule = new SwerveModule(7, 1, WpiSwerveDrive.frontLeftOffset);
             //   m_FrontRightModule = new SwerveModule(4, 3, WpiSwerveDrive.frontRightOffset);
             //   m_BackLeftModule = new SwerveModule(5, 2, WpiSwerveDrive.backLeftOffset);
             //   m_BackRightModule = new SwerveModule(6, 0, WpiSwerveDrive.backRightOffset);
                // #endregion

                // #region Initialize Subsystem Motors
                m_IntakeMotor = new VictorSPX(10);
                m_IntakePnematic = new Solenoid(5);
                m_IntakeDrop = new Solenoid(1);

                m_ConveyorMotorTop = new TalonSRX(14);
                m_ConveyorMotorBottom = new TalonSRX(11);

                // #region Initialize Shooter Motors
                m_TopShooterMotor = new CANSparkMax(MotorControllers.kTopShooter, MotorType.kBrushed);
                m_BottomShooterMotor = new CANSparkMax(MotorControllers.kBottomShooter, MotorType.kBrushed);
                m_TopShooterMotor.setIdleMode(IdleMode.kCoast);
                m_BottomShooterMotor.setIdleMode(IdleMode.kCoast);
                
                // #region Initialize Shooter Pneumatics
                m_LongShooterSolenoid = new Solenoid(4);
                m_ShortShooterSolenoid = new Solenoid(0);
                m_ShooterLights = new Solenoid(7);
                // #endregion

                // #region Initialize Climb Motors
                m_ClimbPullMotor = new TalonSRX(12);
                m_ClimbPullMotor.setNeutralMode(NeutralMode.Brake);

                m_ClimbLiftMotor = new VictorSPX(13);
                // #endregion

                // #endregion

                // #region Initialize and zero gyro
                try {
                        ahrs = new AHRS(SPI.Port.kMXP);
                        ahrs.calibrate();
                        ahrs.zeroYaw();
                } catch (final Exception ex) {
                        DriverStation.reportError("Error instantiating navX MXP:  " + ex.getMessage(), true);
                }
                // #endregion

                // #region Initialize Custom Classes
               
                m_Intake = new Intake(m_IntakeMotor, m_IntakePnematic);

                m_Conveyor = new Conveyor(m_ConveyorMotorTop, m_ConveyorMotorBottom);

                m_Shooter = new Shooter(m_TopShooterMotor, m_BottomShooterMotor, m_LongShooterSolenoid,
                                m_ShortShooterSolenoid);

                m_Climber = new Climber(m_ClimbPullMotor, m_ClimbLiftMotor);

                m_ledStrip = new LEDStrip(LED_PWM_PORT, NUM_LEDS);

                conveyorTimer = new Timer();
                conveyorDelayTimer = new Timer();

                shooterPneumaticTimer = new Timer();
                shooterPneumaticResetTimer = new Timer();

                delaySwitch = new BinarySwitch(0, 1, 2, 3);
                autoChooserSwitch = new BinarySwitch(4, 5, 6, 7);

                autoTimer = new Timer();
                autoSubTimer = new Timer();
                // #endregion

                // #region Get shuffleboard tab
                SwerveTab.add("Front Left Swerve Angle Gyro", frontLeftGyroAdapter)
                                .withProperties(Map.of("Starting angle", 0)).withPosition(3, 0).withSize(5, 5);
                SwerveTab.add("Front Right Swerve Angle Gyro", frontRightGyroAdapter)
                                .withProperties(Map.of("Starting angle", 0)).withPosition(9, 0).withSize(5, 5);
                SwerveTab.add("Back Left Swerve Angle Gyro", backLeftGyroAdapter)
                                .withProperties(Map.of("Starting angle", 0)).withPosition(3, 5).withSize(5, 5);
                SwerveTab.add("Back Right Swerve Angle Gyro", backRightGyroAdapter)
                                .withProperties(Map.of("Starting angle", 0)).withPosition(9, 5).withSize(5, 5);
                // #endregion
        }

        /**
         * This function is called every robot packet, no matter the mode. Use this for
         * items like diagnostics that you want ran during disabled, autonomous,
         * teleoperated and test.
         */
        @Override
        public void robotPeriodic() {
                m_ledStrip.update();
                if (isDisabled()) {
                        if (ahrs.isCalibrating()) {
                                m_ledStrip.setMode(LEDStrip.MODE_SOLID);
                                m_ledStrip.setColor(Colors.RED);
                        } else {
                                m_ledStrip.setMode(LEDStrip.MODE_RAINBOW);

                        }
                }

                // #region shuffleboard constant updates
                if (!shuffleboardRecording && isEnabled()) {
                        Shuffleboard.startRecording();
                        shuffleboardRecording = true;
                } else if (shuffleboardRecording && isDisabled()) {
                        Shuffleboard.stopRecording();
                        shuffleboardRecording = false;
                }

                // #region Update swerve shuffleboard
                
                networkTableEntryFrontLeftEncoderActual.setDouble(m_Drive.getSteerEncoderActual(SwerveModules.kFrontLeft));
                networkTableEntryFrontRightEncoderActual.setDouble(m_Drive.getSteerEncoderActual(SwerveModules.kFrontRight));
                networkTableEntryBackLeftEncoderActual.setDouble(m_Drive.getSteerEncoderActual(SwerveModules.kBackLeft));
                networkTableEntryBackRightEncoderActual.setDouble(m_Drive.getSteerEncoderActual(SwerveModules.kBackRight));

                networkTableEntryFrontLeftEncoderTarget.setDouble(m_Drive.getSteerEncoderTarget(SwerveModules.kFrontLeft));
                networkTableEntryFrontRightEncoderTarget.setDouble(m_Drive.getSteerEncoderTarget(SwerveModules.kFrontRight));
                networkTableEntryBackLeftEncoderTarget.setDouble(m_Drive.getSteerEncoderTarget(SwerveModules.kBackLeft));
                networkTableEntryBackRightEncoderTarget.setDouble(m_Drive.getSteerEncoderTarget(SwerveModules.kBackRight));
                
                frontDriveDistance.setDouble(m_Drive.getDriveEncoder(SwerveModules.kFrontLeft));

                frontLeftAngle.setDouble(m_Drive.getDegrees(SwerveModules.kFrontLeft));
                frontRightAngle.setDouble(m_Drive.getDegrees(SwerveModules.kFrontRight));
                backLeftAngle.setDouble(m_Drive.getDegrees(SwerveModules.kBackLeft));
                backRightAngle.setDouble(m_Drive.getDegrees(SwerveModules.kBackRight));
                
                networkTableEntryJoystickX.setDouble(m_joystickLeft.getRawAxis(0));
                networkTableEntryJoystickY.setDouble(m_joystickLeft.getRawAxis(1));
                networkTableEntryJoystickZ.setDouble(m_joystickRight.getRawAxis(0));

                networkTableEntryFWD.setDouble(m_Drive.getDriveFWD());
                networkTableEntryRCW.setDouble(m_Drive.getDriveRCW());
                networkTableEntrySTR.setDouble(m_Drive.getDriveSTR());

                shuffleboardGyroFused.setDouble(ahrs.getFusedHeading());

                // networkTableEntryConveyorSensor
                // networkTableEntryConveyorShuttoffSensor
                // networkTableEntryConveyorTriggered
                // networkTableEntryConveyorShutoffTriggered

                networkTableEntryConveyorSensor.setDouble((double) conveyorSensor.getValue());
                networkTableEntryConveyorShuttoffSensor.setDouble((double) conveyorShutdownSensor.getValue());
                networkTableEntryConveyorTriggered.setBoolean(conveyorSensor.getValue() < 2000);
                networkTableEntryConveyorShutoffTriggered.setBoolean(conveyorShutdownSensor.getValue() < 2000);

                networkTableEntryVisionDistance.setDouble(Vision.getDistance());
                networkTableEntryShooterSonar.setDouble(shooterSonarSensor.getValue());
                // #endregion
                // #endregion
        }

        /**
         * This function is called when autonomous is first started.
         */
        @Override
        public void autonomousInit() {
                m_Drive.driveKill();
                m_Shooter.shooterKill();
                m_Conveyor.conveyorKill();
                m_Climber.climbKill();
                m_Intake.intakeKill();
                autoTimerHasBeenSet = false;

                switch (autoChooserSwitch.getValue()) {
                        case AutoModes.DoNothing:
                                shouldRun = false;
                                shouldShoot = false;
                                shouldDriveForward = false;
                                shouldDrive = false;
                                shouldMoveFirstPartner = false;
                                shouldMoveSecondPartner = false;
                                break;
                        case AutoModes.OnlyDriveForward:
                                shouldRun = true;
                                shouldShoot = false;
                                shouldDriveForward = true;
                                shouldDrive = true;
                                shouldMoveFirstPartner = false;
                                shouldMoveSecondPartner = false;
                                break;
                        case AutoModes.OnlyDriveBackward:
                                shouldRun = true;
                                shouldShoot = false;
                                shouldDriveForward = false;
                                shouldDrive = true;
                                shouldMoveFirstPartner = false;
                                shouldMoveSecondPartner = false;
                                break;
                        case AutoModes.OnlyShoot:
                                shouldRun = true;
                                shouldShoot = true;
                                shouldDriveForward = false;
                                shouldDrive = false;
                                shouldMoveFirstPartner = false;
                                shouldMoveSecondPartner = false;
                                break;
                        case AutoModes.ShootAndDriveForward:
                                shouldRun = true;
                                shouldShoot = true;
                                shouldDriveForward = true;
                                shouldDrive = true;
                                shouldMoveFirstPartner = false;
                                shouldMoveSecondPartner = false;
                                break;
                        case AutoModes.ShootAndDriveBackward:
                                shouldRun = true;
                                shouldShoot = true;
                                shouldDriveForward = false;
                                shouldDrive = true;
                                shouldMoveFirstPartner = false;
                                shouldMoveSecondPartner = false;
                                break;
                        case AutoModes.ShootAndPushPartnerOne:
                                shouldRun = true;
                                shouldShoot = true;
                                shouldDriveForward = true;
                                shouldDrive = true;
                                shouldMoveFirstPartner = true;
                                shouldMoveSecondPartner = false;
                                break;
                        case AutoModes.ShootAndPushPartnerTwo:
                                shouldRun = true;
                                shouldShoot = true;
                                shouldDriveForward = true;
                                shouldDrive = true;
                                shouldMoveFirstPartner = false;
                                shouldMoveSecondPartner = true;
                                break;
                        case AutoModes.ShootAndPushBothPartners:
                                shouldRun = true;
                                shouldShoot = true;
                                shouldDriveForward = true;
                                shouldDrive = true;
                                shouldMoveFirstPartner = true;
                                shouldMoveSecondPartner = true;
                                break;
                        case AutoModes.PushOnlyPartnerOne:
                                shouldRun = true;
                                shouldShoot = false;
                                shouldDriveForward = true;
                                shouldDrive = true;
                                shouldMoveFirstPartner = true;
                                shouldMoveSecondPartner = false;
                                break;
                        default:
                                shouldRun = true;
                                shouldShoot = false;
                                shouldDriveForward = true;
                                shouldDrive = true;
                                shouldMoveFirstPartner = false;
                                shouldMoveSecondPartner = false;
                                break;
                }

                delay = delaySwitch.getValue();

                // if (isSimulation()) {
                // shouldRun = true;
                // shouldShoot = false;
                // shouldDriveForward = true;
                // shouldDrive = true;
                // shouldMoveFirstPartner = true;
                // shouldMoveSecondPartner = true;
                // delay = 0;
                // }

                if (shouldRun) {
                        if (shouldShoot) {
                                maxDelay += shooterSpinupTime + shooterLoadTime;
                        }
                        if (shouldDrive) {
                                maxDelay += driveForwardTime;
                                if (shouldMoveFirstPartner) {
                                        maxDelay += driveResetTime + pushBackTime;
                                }
                                if (shouldMoveSecondPartner) {
                                        maxDelay += secondPullForwardtime + secondPushBackTime + driveStrafeTime
                                                        + moveForwardTime;
                                }
                        }

                }

                delay = delay > maxDelay ? maxDelay : delay;

                autoSubTimerHasBeenSet = false;
        }

        /**
         * This function is called periodically during autonomous.
         */
        @Override
        public void autonomousPeriodic() {
                m_Intake.raise();
                m_Shooter.longPneumatic(LongUp);
                m_ShortShooterSolenoid.set(ShortDown);

                if (shouldRunShooter) {
                        m_Shooter.voltageRun(.7);
                } else {
                        m_Shooter.shooterKill();
                }

                switch (currentState) {
                        case AutoEnumeration.Delay:
                                if (delay == 0) {
                                        currentState = AutoEnumeration.SpinupShooter;
                                } else {
                                        if (!autoSubTimerHasBeenSet) {
                                                autoSubTimer.reset();
                                                autoSubTimer.start();
                                                autoSubTimerHasBeenSet = true;
                                        }
                                        if (autoSubTimer.hasPeriodPassed(delay)) {
                                                currentState = AutoEnumeration.SpinupShooter;
                                                autoSubTimerHasBeenSet = false;
                                        }
                                }
                                break;
                        case AutoEnumeration.SpinupShooter:
                                if (shouldShoot) {
                                        if (!autoSubTimerHasBeenSet) {
                                                autoSubTimer.reset();
                                                autoSubTimer.start();
                                                autoSubTimerHasBeenSet = true;
                                        }
                                        if (autoSubTimer.hasPeriodPassed(shooterSpinupTime)) {
                                                currentState = AutoEnumeration.LoadShooter;
                                                autoSubTimerHasBeenSet = false;
                                        } else {
                                                shouldRunShooter = true;
                                                m_Shooter.voltageRun(.87); // Auto Shooting Power

                                        }
                                } else {
                                        currentState = AutoEnumeration.StopShooting;
                                }
                                break;
                        case AutoEnumeration.LoadShooter:
                                if (!autoSubTimerHasBeenSet) {
                                        autoSubTimer.reset();
                                        autoSubTimer.start();
                                        autoSubTimerHasBeenSet = true;
                                }
                                if (autoSubTimer.hasPeriodPassed(shooterLoadTime)) {
                                        currentState = AutoEnumeration.StopShooting;
                                        autoSubTimerHasBeenSet = false;
                                } else {
                                        m_Conveyor.loadConveyor(shooterLoadSpeed);
                                }
                                break;
                        case AutoEnumeration.StopShooting:
                                m_Shooter.shooterKill();
                                shouldRunShooter = false;
                                m_Conveyor.conveyorKill();
                                if (shouldDrive) {
                                        currentState = AutoEnumeration.MoveFirstRobot;
                                } else {
                                        currentState = AutoEnumeration.End;
                                }
                                break;
                        case AutoEnumeration.MoveFirstRobot:
                                if (shouldMoveFirstPartner) {
                                        shouldDriveForward = true;
                                        if (!autoSubTimerHasBeenSet) {
                                                autoSubTimer.reset();
                                                autoSubTimer.start();
                                                autoSubTimerHasBeenSet = true;
                                        }
                                        if (autoSubTimer.get() < pushBackTime) {
                                                m_Drive.driveWithJoystick(0, -drivePushPower, 0, true, false);
                                        } else if (autoSubTimer.get() > pushBackTime
                                                        && autoSubTimer.get() < driveResetTime + pushBackTime) {
                                                m_Drive.driveWithJoystick(0, -driveResetPower, 0, true, false);
                                        } else {
                                                currentState = AutoEnumeration.MoveSecondRobot;
                                                autoSubTimerHasBeenSet = false;
                                        }
                                } else {
                                        currentState = AutoEnumeration.MoveSecondRobot;
                                        autoSubTimerHasBeenSet = false;
                                 }
                                break;
                        case AutoEnumeration.MoveSecondRobot:
                                if (shouldMoveSecondPartner) {
                                        shouldDriveForward = true;
                                        if (!autoSubTimerHasBeenSet) {
                                                autoSubTimer.reset();
                                                autoSubTimer.start();
                                                autoSubTimerHasBeenSet = true;
                                        }
                                        if (autoSubTimer.get() < moveForwardTime) {
                                                m_Drive.driveWithJoystick(0, -driveResetPower, 0, true, false);
                                        } else if (autoSubTimer.get() > moveForwardTime
                                                        && autoSubTimer.get() < driveStrafeTime + moveForwardTime) {
                                                m_Drive.driveWithJoystick(0, 0, -strafePower, true, false);
                                        } else if (autoSubTimer.get() > driveStrafeTime + moveForwardTime
                                                        && autoSubTimer.get() < secondPushBackTime + driveStrafeTime
                                                                        + moveForwardTime) {
                                                m_Drive.driveWithJoystick(0, -drivePushPower, 0, true, false);
                                        } else if (autoSubTimer.get() > secondPushBackTime + driveStrafeTime
                                                        + moveForwardTime
                                                        && autoSubTimer.get() < secondPullForwardtime
                                                                        + secondPushBackTime + driveStrafeTime
                                                                        + moveForwardTime) {
                                                m_Drive.driveWithJoystick(0, -driveResetPower, 0, true, false);
                                        } else {
                                                currentState = AutoEnumeration.Drive;
                                                autoSubTimerHasBeenSet = false;
                                        }
                                } else {
                                        currentState = AutoEnumeration.Drive;
                                        autoSubTimerHasBeenSet = false;
                                }
                                break;
                        case AutoEnumeration.Drive:
                                if (shouldDrive) {
                                        if (!autoSubTimerHasBeenSet) {
                                                autoSubTimer.reset();
                                                autoSubTimer.start();
                                                autoSubTimerHasBeenSet = true;
                                        }
                                        if (autoSubTimer.hasPeriodPassed(driveForwardTime)) {
                                                autoSubTimerHasBeenSet = false;
                                                currentState = AutoEnumeration.End;
                                        } else {
                                                double drivePower = 0.6;
                                                drivePower *= shouldDriveForward ? -1 : 1;
                                                m_Drive.driveWithJoystick(0, drivePower, 0, true, false);
                                        }
                                }
                                break;
                        case AutoEnumeration.End:
                                m_Shooter.shooterKill();
                                shouldRunShooter = false;
                                m_Conveyor.conveyorKill();
                                m_Drive.driveWithJoystick(0, 0, 0, true, false);
                                break;
                        default:
                                DriverStation.reportError("Hit default auto case", false);
                                shouldRunShooter = false;
                                m_Shooter.shooterKill();
                                m_Conveyor.conveyorKill();
                                m_Drive.driveWithJoystick(0, 0, 0, true, false);
                                break;
                }
        }
        
        @Override
        public void teleopInit() {
        }
        /**
         * This function is called periodically during operator control.
         */
        @Override
        public void teleopPeriodic() {

                m_gamepadY = m_gamepad.getRawButton(4);
                m_gamepadX = m_gamepad.getRawButton(3);
                m_gamepadA = m_gamepad.getRawButton(1);
                m_gamepadB = m_gamepad.getRawButton(2);
                m_gamepadRightBumper = m_gamepad.getRawButton(6);
                m_gamepadLeftBumper = m_gamepad.getRawButton(5);
                m_gamepadLeftTrigger = m_gamepad.getRawAxis(2);
                m_gamepadRightTrigger = m_gamepad.getRawAxis(3);
                m_gamepadRightJoystickY = m_gamepad.getRawAxis(5);
                m_gamepadLeftJoystickY = m_gamepad.getRawAxis(1);

                if (m_gamepad.getRawButton(1)) {
                        m_IntakeDrop.set(true);
                }

                // #region Swerve Drive Code
                try {
                        // Get gryo angle
                        gyroAngle = ahrs.getFusedHeading();
                        // fieldCentric is true only if you can recieve a gyro angle
                        fieldCentric = true;
                } catch (final Exception e) {
                        DriverStation.reportError("Couldn't recieve gyro angle:" + e, true);
                        DriverStation.reportError("Moving to robotCentric Controls", true);
                        gyroAngle = 0;
                }

                // Run Drive
                // if you change joysticks or want to pull different Axis change them here
                final double strafe = m_joystickLeft.getRawAxis(0);
                final double forward = m_joystickLeft.getRawAxis(1);
                final double rotateClockwise = m_joystickRight.getRawAxis(0);

                //KK 1/23/21 disabled field centric
                m_Drive.driveWithJoystick(rotateClockwise, forward, strafe, false, false);
                //m_Drive.driveWithJoystick(0,-1,.5,false,false);
               
                //KK 1/25/21 Drive wheel TableStringConverter
                if (m_joystickLeft.getRawButton(8)){
                         m_Drive.move((double)0,(double)0.2,(double) 3,true);
                        }
                if (m_joystickLeft.getRawButton(9)){
                       //Joe's Auto
                        m_Drive.move((double)0,(double).8,(double)10,false);
                        m_Drive.move((double)45,(double).6,(double)3,false);
                        m_Drive.move((double)0,(double).8,(double)3,false);
                        m_Drive.move((double)45,(double).6,(double).45,false);
                        m_Drive.move((double)0,(double).7,(double)2,true);
                        
                        }
        
                if (m_joystickLeft.getRawButton(10)){
                        m_Drive.moveWheel(SwerveModules.kBackLeft,0.1);
                }
                else{
                        m_Drive.moveWheel(SwerveModules.kBackLeft,0.0);
                                      }
                        
                if (m_joystickLeft.getRawButton(11)){
                                m_Drive.moveWheel(SwerveModules.kBackRight,0.1);
                        }
                        else{
                                m_Drive.moveWheel(SwerveModules.kBackRight,0.0);
                        }
            
                // DriverStation.reportError(rotateClockwise + "", false);

                // shuffleboardGyroCompass.setDouble(gyroAngle);

                isShootingLong = m_joystickRight.getRawButton(1);
                isShootingShort = m_joystickLeft.getRawButton(2);

                m_ledStrip.setMode(LEDStrip.MODE_WAVE);
                LED_Color = Colors.ORANGE;
                m_ShooterLights.set(off);

                // Vision.setDriverMode(!isShooting);

                if (isShootingLong) {
                        m_ledStrip.setMode(LEDStrip.MODE_BAR);
                        m_ledStrip.setPercentage(Vision.getLedValue());
                        m_ShooterLights.set(on);

                        // Shooter Motor
                        m_Shooter.voltageRun(0.8);

                        // Pneumatics
                        m_LongShooterSolenoid.set(LongUp); // this is actually up
                        m_ShortShooterSolenoid.set(ShortDown);
                } else if (isShootingShort) {
                        // Leds
                        m_ledStrip.setMode(LEDStrip.MODE_SOLID);
                        LED_Color = Colors.GREEN;

                        // Shooter Motor
                        m_Shooter.voltageRun(0.6);

                        // Pneumatics
                        if (!shooterPneumaticTimerSet) {
                                shooterPneumaticTimer.reset();
                                shooterPneumaticTimer.start();
                                shooterPneumaticTimerSet = true;
                        }
                        if (shooterPneumaticTimer.get() < 0.5) {
                                // m_Drive.driveWithJoystick(rotateClockwise, 0.0, strafe, fieldCentric, false);
                                m_LongShooterSolenoid.set(LongUp);
                                m_ShortShooterSolenoid.set(ShortDown);
                        } else if (shooterPneumaticTimer.get() > 0.5) {
                                m_LongShooterSolenoid.set(LongDown);
                                m_ShortShooterSolenoid.set(ShortUp);
                        }
                        shooterPneumaticRestTimerSet = true;
                        shooterPneumaticResetTimer.reset();
                        shooterPneumaticResetTimer.start();
                } else {

                        m_LongShooterSolenoid.set(LongUp);
                        // m_ShortShooterSolenoid.set(ShortDown);

                        if (shooterPneumaticRestTimerSet) {

                                shooterPneumaticRestTimerSet = true;
                                if (shooterPneumaticResetTimer.get() < 0.5) {
                                        m_LongShooterSolenoid.set(LongUp);
                                        m_ShortShooterSolenoid.set(ShortDown);

                                } else if ((shooterPneumaticResetTimer.get() > 0.5)) {
                                        m_LongShooterSolenoid.set(LongDown);
                                        m_ShortShooterSolenoid.set(ShortDown);
                                        shooterPneumaticTimerSet = false;

                                        m_Shooter.shooterKill();

                                }
                        } else {
                                // Leds
                                m_ledStrip.setMode(LEDStrip.MODE_WAVE);
                                LED_Color = Colors.ORANGE;
                                m_ShooterLights.set(off);

                                // Shooter Motor
                                m_Shooter.shooterKill();

                                // Pneumatics
                                m_LongShooterSolenoid.set(LongDown);
                                m_ShortShooterSolenoid.set(ShortDown);
                        }

                }

                m_ledStrip.setColor(LED_Color);
            
                // if (isShootingLong && Vision.getIsValid() && false) {
                // m_Drive.driveWithJoystick(Vision.getTurnValue(), forward, strafe,
                // fieldCentric,
                // isShootingLong);
                // }
                // else if (m_joystickLeft.getRawButton(1)) {
                // m_Drive.driveWithJoystick(Vision.getBallTurnValue(), 0, 0, fieldCentric,
                // true);
                // }
                // else {
                // m_Drive.driveWithJoystick(rotateClockwise, forward, strafe, fieldCentric,
                // false);
                // }

                // using a Gyro Widget on the Shuffleboard, have to convert our angle 180 ->
                // -180 to 0 - 360
                // then put the converted angle into the Gyro Adapter widget.
                frontLeftGyroAdapter.setAngle(m_Drive.getSteerEncoderActual(SwerveModules.kFrontLeft));
                frontRightGyroAdapter.setAngle(m_Drive.getSteerEncoderActual(SwerveModules.kFrontRight));
                backLeftGyroAdapter.setAngle(m_Drive.getSteerEncoderActual(SwerveModules.kBackLeft));
                backRightGyroAdapter.setAngle(m_Drive.getSteerEncoderActual(SwerveModules.kBackRight));
                //frontLeftGyroAdapter.setAngle(m_Drive.ShuffleBoardAngleConversion(m_Drive.frontLeftAngle))
                //frontRightGyroAdapter.setAngle(m_Drive.ShuffleBoardAngleConversion(m_Drive.frontRightAngle));
                //backLeftGyroAdapter.setAngle(m_Drive.ShuffleBoardAngleConversion(m_Drive.backLeftAngle));
                // backRightGyroAdapter.setAngle(m_Drive.ShuffleBoardAngleConversion(m_Drive.backRightAngle));
                
                networkTableEntryFrontLeftSpeed.setDouble(m_Drive.getSpeed(SwerveModules.kFrontLeft));
                networkTableEntryFrontRightSpeed.setDouble(m_Drive.getSpeed(SwerveModules.kFrontRight));
                networkTableEntryBackLeftSpeed.setDouble(m_Drive.getSpeed(SwerveModules.kBackLeft));
                networkTableEntryBackRightSpeed.setDouble(m_Drive.getSpeed(SwerveModules.kBackRight));

                // #region Subsystem Controls

                // #region Intake control

                if (m_joystickRight.getRawButton(3) || m_gamepadX) {
                        m_Intake.intake(.75);
                        m_Intake.lower();
                } else if (m_joystickRight.getRawButton(4) || m_joystickRight.getRawButton(5)) {
                        m_Intake.intake(.5);
                        m_Intake.lower();
                } else if (m_joystickLeft.getRawButton(3) || m_gamepadB) {
                        m_Intake.outtake(1.0);
                        m_Intake.lower();
                } else if (m_joystickRight.getRawButton(9) || m_gamepadRightBumper || m_joystickLeft.getRawButton(4)
                                || m_joystickLeft.getRawButton(5)) {
                        m_Intake.raise();
                } else {
                        m_Intake.intakeKill();
                }
                // #endregion

                // #region Conveyor control

                conveyorSensorTriggered = conveyorSensor.getValue() < 2000;
                conveyorShutdownTriggered = conveyorShutdownSensor.getValue() < 2000;
                // conveyorSensorTriggered = false;

                if (conveyorSensorTriggered) {
                        conveyorIsLoaded = true;
                }

                if (m_joystickRight.getRawButton(2)) {
                        conveyorSensorTriggered = false;
                        conveyorShutdownTriggered = false;
                        conveyorButtonHasBeenPressed = true;
                        m_Conveyor.feedShooter();
                } else if (m_joystickLeft.getRawButton(7)) {
                        conveyorSensorTriggered = false;
                        conveyorShutdownTriggered = false;
                        m_Conveyor.runReverse();
                } else if (!conveyorButtonHasBeenPressed) {
                        if (conveyorShutdownTriggered) {
                                // m_Intake.intakeKill();
                                m_Conveyor.conveyorKill();
                        } else {
                                if (conveyorSensorTriggered) {
                                        // m_Intake.intake(.5);
                                        m_Conveyor.loadConveyor();
                                        conveyorIsLoaded = true;
                                } else {
                                        if (conveyorTimerHasBeenSet) {
                                                if (conveyorDelayTimer.hasPeriodPassed(0.2)) {
                                                        m_Conveyor.conveyorKill();
                                                        conveyorTimerHasBeenSet = false;
                                                        conveyorIsLoaded = false;
                                                } else {
                                                        m_Conveyor.loadConveyor();
                                                }
                                        } else if (conveyorIsLoaded) {
                                                conveyorDelayTimer.reset();
                                                conveyorDelayTimer.start();
                                                conveyorTimerHasBeenSet = true;
                                        }
                                        // m_Intake.intakeKill();
                                }
                        }
                } else {
                        if (!m_joystickRight.getRawButton(2) && !m_joystickLeft.getRawButton(7)) {
                                m_Conveyor.conveyorKill();
                                conveyorButtonHasBeenPressed = false;
                        }
                }
                // #endregion

                // #region Climber control
                climbPullPower = m_gamepad.getRawAxis(1);
                climbRaisePower = m_gamepad.getRawAxis(5);

                if (isClimbing) {
                        if (UtilFuncs.isOutOfDeadzone(climbPullPower, climbDeadzone)) {
                                m_Climber.CLIMBCLIMBCLIMB(UtilFuncs.motorPowerClip(climbPullPower));
                        } else {
                                m_Climber.climbPullKill();
                        }

                        if (UtilFuncs.isOutOfDeadzone(climbRaisePower, climbDeadzone)) {
                                m_Climber.climbRaise(UtilFuncs.motorPowerClip(climbRaisePower));
                        } else {
                                m_Climber.climbRaiseKill();
                        }
                } else {
                        m_Climber.climbKill();
                }
                // #endregion

                // #endregion
        }

        /**
         * This function is called periodically during test mode.
         */
        @Override
        public void testPeriodic() {
        }

        @Override
        public void disabledInit() {

        }
}
