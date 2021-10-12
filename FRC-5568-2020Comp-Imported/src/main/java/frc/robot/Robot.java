/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.Map;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import frc.robot.classes.AutoEnumeration;
import frc.robot.classes.AutoModes;
import frc.robot.classes.BinarySwitch;
import frc.robot.classes.Climber;
import frc.robot.classes.Colors;
import frc.robot.classes.Constants;
import frc.robot.classes.Conveyor;
import frc.robot.classes.DistanceSensor;
import frc.robot.classes.Intake;
import frc.robot.classes.LEDStrip;
import frc.robot.classes.Shooter;
import frc.robot.classes.SwerveDrive;
import frc.robot.classes.SwerveGyroAdapter;

import frc.robot.classes.UtilFuncs;
import frc.robot.classes.Vision;
import frc.robot.classes.Limelight;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
public class Robot extends TimedRobot {

        // Create Joysticks
        Joystick m_joystickLeft;
        Joystick m_joystickRight;
        Joystick m_gamepad;


        // Create Drive Motors
        TalonSRX m_FrontLeftSteering;
        TalonSRX m_FrontRightSteering;
        TalonSRX m_BackLeftSteering;
        TalonSRX m_BackRightSteering;
        CANSparkMax m_FrontLeftDrive;
        CANSparkMax m_FrontRightDrive;
        CANSparkMax m_BackLeftDrive;
        CANSparkMax m_BackRightDrive;

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
        AHRS m_ahrs;

        // Create Shooting solenoids
        Solenoid m_LongShooterSolenoid;
        Solenoid m_ShortShooterSolenoid;
        Solenoid m_ShooterLights;

        // Create Custom Classes
        SwerveDrive m_Drive;
        Intake m_Intake;
        Conveyor m_Conveyor;
        Shooter m_Shooter;
        Climber m_Climber;
        LEDStrip m_ledStrip;
        DistanceSensor m_DistanceSensor;
        Ultrasonic m_FrontUltraSonic= new Ultrasonic(8,9);
        

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
        double m_LongShotSpeed=Constants.kShootingTrench;
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
        boolean shouldAutoNav1 = false;
        boolean shouldAutoNav2 = false;
        boolean shouldRunShooter = false;

        double shooterSpinupTime = 2.80;
        double shooterLoadTime = 6.0;
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

        boolean ShortUp = true;
        boolean ShortDown = false;
        boolean LongUp = false;
        boolean LongDown = true;
        boolean on = true;
        boolean off = false;

        private final Limelight m_LimeLight = new Limelight(Constants.LimeLightValues.targetHeight, Constants.LimeLightValues.limelightHeight, Constants.LimeLightValues.limelightAngle,Constants.kShootingTrench);
  
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

        PowerDistributionPanel m_robotPDP = new PowerDistributionPanel(0);

        // #region NetworkEntries
        // Create Network Table Entries

        static NetworkTableEntry networkTableEntryShooterSonar = SubSystems.add("Shooter Sonar", 0)
                        .withWidget(BuiltInWidgets.kNumberBar).withSize(2, 2).getEntry();

        static NetworkTableEntry networkTableEntrySonarShortShot = SubSystems.add("Short Shot",false)
                .withWidget(BuiltInWidgets.kBooleanBox).withSize(2, 2).getEntry();
        
        static NetworkTableEntry networkTableEntryShooterPower = AutoTab.add("Shooter Power", 0)
                        .withWidget(BuiltInWidgets.kNumberSlider).withSize(2, 2).withProperties(Map.of("min", 0, "max", 1,"block increment",0.05)).getEntry();

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

        static NetworkTableEntry frontLeft360Angle = SwerveEncoders.add("FL 360 Angle", 0)
                        .withWidget(BuiltInWidgets.kTextView).withPosition(4, 3).withSize(2, 1).getEntry();
        static NetworkTableEntry frontRight360Angle = SwerveEncoders.add("FR 360 Angle", 0)
                        .withWidget(BuiltInWidgets.kTextView).withPosition(6, 3).withSize(2, 1).getEntry();
        static NetworkTableEntry backLeft360Angle = SwerveEncoders.add("BL 360 Angle", 0)
                        .withWidget(BuiltInWidgets.kTextView).withPosition(4, 4).withSize(2, 1).getEntry();
        static NetworkTableEntry backRight360Angle = SwerveEncoders.add("BR 360 Angle", 0)
                        .withWidget(BuiltInWidgets.kTextView).withPosition(6, 4).withSize(2, 1).getEntry();

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
                m_LimeLight.turnLEDOff();
                // #region Initialize Joysticks
                m_joystickLeft = new Joystick(0);
                m_joystickRight = new Joystick(1);
                m_gamepad = new Joystick(2);
                // #endregion

                // #region Initialize Drive Motors
                m_FrontLeftSteering = new TalonSRX(1);
                m_FrontRightSteering = new TalonSRX(3);
                m_BackLeftSteering = new TalonSRX(2);
                m_BackRightSteering = new TalonSRX(0);
                m_FrontLeftDrive = new CANSparkMax(7, MotorType.kBrushless);
                m_FrontRightDrive = new CANSparkMax(4, MotorType.kBrushless);
                m_BackLeftDrive = new CANSparkMax(5, MotorType.kBrushless);
                m_BackRightDrive = new CANSparkMax(6, MotorType.kBrushless);
                // #endregion

                // #region Initialize Subsystem Motors
                m_IntakeMotor = new VictorSPX(10);
                m_IntakePnematic = new Solenoid(5);
                m_IntakeDrop = new Solenoid(1);

                m_ConveyorMotorTop = new TalonSRX(14);
                m_ConveyorMotorBottom = new TalonSRX(11);

                // #region Initialize Shooter Motors
                m_TopShooterMotor = new CANSparkMax(9, MotorType.kBrushed);
                m_BottomShooterMotor = new CANSparkMax(8, MotorType.kBrushed);

                // #region Initialize Shooter Pneumatics
                m_LongShooterSolenoid = new Solenoid(4);
                m_ShortShooterSolenoid = new Solenoid(0);
                m_ShooterLights = new Solenoid(7);
                // #endregion

                // #region Initialize Climb Motors
                m_ClimbPullMotor = new TalonSRX(12);
                m_ClimbLiftMotor = new VictorSPX(13);
                // #endregion

                // #endregion
                m_FrontUltraSonic.setAutomaticMode(true);
                // #region Initialize and zero gyro
                try {
                        m_ahrs = new AHRS(SPI.Port.kMXP);
                        m_ahrs.calibrate();
                        m_ahrs.zeroYaw();
                } catch (final Exception ex) {
                        DriverStation.reportError("Error instantiating navX MXP:  " + ex.getMessage(), true);
                }
                // #endregion

                // #region Initialize Custom Classes
                m_Drive = new SwerveDrive(m_FrontLeftSteering, m_FrontRightSteering, m_BackLeftSteering,
                                m_BackRightSteering, m_FrontLeftDrive, m_FrontRightDrive, m_BackLeftDrive,
                                m_BackRightDrive, m_ahrs);

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
                m_LimeLight.update();
        }

        /**
         * This function is called every robot packet, no matter the mode. Use this for
         * items like diagnostics that you want ran during disabled, autonomous,
         * teleoperated and test.
         */
        @Override
        public void robotPeriodic() {
                boolean onSonarTarget=false;
                if(m_robotPDP.getTotalCurrent()>2){
           //     System.out.println(m_robotPDP.getCurrent(1));
                if(m_robotPDP.getCurrent(1)<10.0 && m_robotPDP.getCurrent(1)>6.0){
                System.out.println("FULL SPEED");
                }
                }       
                m_LimeLight.update();
                m_ShooterLights.set(on);
                m_ledStrip.update();
                if (isDisabled()) {
                        if (m_ahrs.isCalibrating()) {
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
                networkTableEntryFrontLeftEncoderActual.setDouble(m_FrontLeftSteering.getSelectedSensorPosition());
                networkTableEntryFrontRightEncoderActual.setDouble(m_FrontRightSteering.getSelectedSensorPosition());
                networkTableEntryBackLeftEncoderActual.setDouble(m_BackLeftSteering.getSelectedSensorPosition());
                networkTableEntryBackRightEncoderActual.setDouble(m_BackRightSteering.getSelectedSensorPosition());

                networkTableEntryFrontLeftEncoderTarget.setDouble(m_Drive.frontLeftTargetPosition);
                networkTableEntryFrontRightEncoderTarget.setDouble(m_Drive.frontRightTargetPosition);
                networkTableEntryBackLeftEncoderTarget.setDouble(m_Drive.backLeftTargetPosition);
                networkTableEntryBackRightEncoderTarget.setDouble(m_Drive.backRightTargetPosition);

                frontLeftAngle.setDouble(m_Drive.frontLeftAngle);
                frontRightAngle.setDouble(m_Drive.frontRightAngle);
                backLeftAngle.setDouble(m_Drive.backLeftAngle);
                backRightAngle.setDouble(m_Drive.backRightAngle);

                frontLeft360Angle.setDouble(m_Drive.frontLeft360Angle);
                frontRight360Angle.setDouble(m_Drive.frontRight360Angle);
                backLeft360Angle.setDouble(m_Drive.backLeft360Angle);
                backRight360Angle.setDouble(m_Drive.backRight360Angle);

                networkTableEntryJoystickX.setDouble(m_joystickLeft.getRawAxis(0));
                networkTableEntryJoystickY.setDouble(m_joystickLeft.getRawAxis(1));
                networkTableEntryJoystickZ.setDouble(m_joystickRight.getRawAxis(0));

                networkTableEntryFWD.setDouble(m_Drive.m_FWD);
                networkTableEntryRCW.setDouble(m_Drive.m_RCW);
                networkTableEntrySTR.setDouble(m_Drive.m_STR);

                shuffleboardGyroFused.setDouble(m_ahrs.getFusedHeading());

                // networkTableEntryConveyorSensor
                // networkTableEntryConveyorShuttoffSensor
                // networkTableEntryConveyorTriggered
                // networkTableEntryConveyorShutoffTriggered

                networkTableEntryConveyorSensor.setDouble((double) conveyorSensor.getValue());
                networkTableEntryConveyorShuttoffSensor.setDouble((double) conveyorShutdownSensor.getValue());
                networkTableEntryConveyorTriggered.setBoolean(conveyorSensor.getValue() < 2000);
                networkTableEntryConveyorShutoffTriggered.setBoolean(conveyorShutdownSensor.getValue() < 2000);

                networkTableEntryVisionDistance.setDouble(Vision.getDistance());
                //netw networkTableEntryShooterSonarorkTableEntryShooterSonar.setDouble(shooterSonarSensor.getValue());
                networkTableEntryShooterSonar.setDouble(m_FrontUltraSonic.getRangeInches());
                //if(m_FrontUltraSonic.isRangeValid()){
                        onSonarTarget=(m_FrontUltraSonic.getRangeInches()>=Constants.kshortShotRangeLow && m_FrontUltraSonic.getRangeInches()<=Constants.kshortShotRangeHigh);
      
                //}                
                networkTableEntrySonarShortShot.setBoolean(onSonarTarget);
                
                // #endregion
                // #endregion
        }

        /**
         * This function is called when autonomous is first started.
         */
        @Override
        public void autonomousInit() {
                m_LimeLight.turnLEDOn();
                m_Drive.driveKill();
                m_Shooter.shooterKill();
                m_Conveyor.conveyorKill();
                m_Climber.climbKill();
                m_Intake.intakeKill();
                currentState = AutoEnumeration.Delay;

                autoTimerHasBeenSet = false;
                //KK 5/29 SWITCH issue delay harcoded to 5 and using delay for auto
                System.out.println("Auto Value Set to " + delaySwitch.getValue());
                //switch (autoChooserSwitch.getValue()) {
                switch (delaySwitch.getValue()) {
                
                        case AutoModes.AutoNav1:
                        shouldRun = false;
                        shouldShoot = false;
                        shouldDriveForward = false;
                        shouldDrive = false;
                        shouldMoveFirstPartner = false;
                        shouldMoveSecondPartner = false;
                        shouldAutoNav1 = true;
                        currentState = AutoEnumeration.End;
                        break;
                case AutoModes.AutoNav2:
                        shouldRun = false;
                        shouldShoot = false;
                        shouldDriveForward = false;
                        shouldDrive = false;
                        shouldMoveFirstPartner = false;
                        shouldMoveSecondPartner = false;
                        shouldAutoNav2 = true;
                        currentState = AutoEnumeration.End;
                        break;
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
                //KK 5/29 SWITCH issue delay harcoded to 2
                //delay = delaySwitch.getValue();
                delay=2;
                System.out.println("Delay Value Set to " + delay);
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
                m_LimeLight.update();
                
                m_Intake.raise();
                m_Shooter.longPneumatic(LongUp);

                m_ShortShooterSolenoid.set(ShortDown);

                if (shouldRunShooter) {
                        m_Shooter.voltageRun(Constants.kAutoShootSpeed);
                } else {
                        m_Shooter.shooterKill();
                }
                if (shouldAutoNav1) {
                        shouldAutoNav1 = false;
                        m_Drive.move((double) 0, (double) .2, (double) 200, false);
                        m_Drive.move((double) 270, (double) .2, (double) 60, false);
                        m_Drive.move((double) 180, (double) .2, (double) 50, false);
                        m_Drive.move((double) 90, (double) .2, (double) 65, false);
                        m_Drive.move((double) 0, (double) .2, (double) 148, false);
                        m_Drive.move((double) 90, (double) .2, (double) 50, false);
                        m_Drive.move((double) 180, (double) .2, (double) 70, false);
                        m_Drive.move((double) 285, (double) .2, (double) 120, false);
                        m_Drive.move((double) 0, (double) .2, (double) 140, false);
                        m_Drive.move((double) 90, (double) .2, (double) 50, false);
                        m_Drive.move((double) 180, (double) .2, (double) 50, false);
                        m_Drive.move((double) 180, (double) .5, (double) 250, false);
                        m_Drive.move((double) 180, (double) .2, (double) 50, false);

                        // m_Drive.move((double)180,(double).2,(double)180,false);
                        /*
                         * m_Drive.move((double)45,(double).2,(double)48,false);
                         * m_Drive.move((double)45,(double).2,(double)50,false);
                         * m_Drive.move((double)45,(double).2,(double)15,false);
                         * m_Drive.move((double)90,(double).2,(double)18,false);
                         * m_Drive.move((double)300,(double).2,(double)16,false);
                         * m_Drive.move((double)180,(double).2,(double)150,false); //
                         * m_Drive.move((double)0,(double).2,(double)0,false);
                         * //m_Drive.move((double)0,(double).2,(double)0,false); //
                         * m_Drive.move((double)0,(double).2,(double)0,false); //
                         * m_Drive.move((double)0,(double).2,(double)0,false); //
                         * m_Drive.move((double)0,(double).2,(double)0,false); //
                         * m_Drive.move((double)0,(double).2,(double)0,false);
                         */
                }
                if (shouldAutoNav2) {
                        shouldAutoNav1 = false;
                        m_Drive.move((double) 0, (double) .2, (double) 3, false);
                        m_Drive.move((double) 315, (double) .2, (double) 20, false);
                        m_Drive.move((double) 45, (double) .2, (double) 25, false);
                        m_Drive.move((double) 315, (double) .2, (double) 5, false);
                        m_Drive.move((double) 270, (double) .2, (double) 5, false);
                        m_Drive.move((double) 225, (double) .2, (double) 5, false);
                        m_Drive.move((double) 135, (double) .2, (double) 25, false);
                        m_Drive.move((double) 225, (double) .2, (double) 20, false);
                        m_Drive.move((double) 180, (double) .2, (double) 4, false);
                        m_Drive.move((double) 0, (double) .2, (double) 0, false);
                        m_Drive.move((double) 0, (double) .2, (double) 0, false);
                        m_Drive.move((double) 0, (double) .2, (double) 0, false);
                        m_Drive.move((double) 0, (double) .2, (double) 0, false);
                        m_Drive.move((double) 0, (double) .2, (double) 0, false);
                        m_Drive.move((double) 0, (double) .2, (double) 0, false);
                        m_Drive.move((double) 0, (double) .2, (double) 0, false);
                        m_Drive.move((double) 0, (double) .2, (double) 0, false);
                        m_Drive.move((double) 0, (double) .2, (double) 0, false);
                        m_Drive.move((double) 0, (double) .2, (double) 0, false);
                        m_Drive.move((double) 0, (double) .2, (double) 0, false);
                        m_Drive.move((double) 0, (double) .2, (double) 0, false);
                        m_Drive.move((double) 0, (double) .2, (double) 0, false);
                        m_Drive.move((double) 0, (double) .2, (double) 0, false);
                        m_Drive.move((double) 0, (double) .2, (double) 0, false);
                        m_Drive.move((double) 0, (double) .2, (double) 0, false);
                        m_Drive.move((double) 0, (double) .2, (double) 0, false);
                        m_Drive.move((double) 0, (double) .2, (double) 0, false);
                        m_Drive.move((double) 0, (double) .2, (double) 0, false);
                        m_Drive.move((double) 0, (double) .2, (double) 0, false);
                        m_Drive.move((double) 0, (double) .2, (double) 0, false);
                        m_Drive.move((double) 0, (double) .2, (double) 0, false);
                        m_Drive.move((double) 0, (double) .2, (double) 0, false);
                        m_Drive.move((double) 0, (double) .2, (double) 0, false);
                        m_Drive.move((double) 0, (double) .2, (double) 0, false);
                        m_Drive.move((double) 0, (double) .2, (double) 0, false);
                        m_Drive.move((double) 0, (double) .2, (double) 0, false);
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

                                        m_Shooter.voltageRun(Constants.kAutoShootSpeed); // Auto Shooting Power was .87
                                                                                         // 2/27/21

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
                                        m_Drive.drive(0, -drivePushPower, 0, true, false);
                                } else if (autoSubTimer.get() > pushBackTime
                                                && autoSubTimer.get() < driveResetTime + pushBackTime) {
                                        m_Drive.drive(0, -driveResetPower, 0, true, false);
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
                                        m_Drive.drive(0, -driveResetPower, 0, true, false);
                                } else if (autoSubTimer.get() > moveForwardTime
                                                && autoSubTimer.get() < driveStrafeTime + moveForwardTime) {
                                        m_Drive.drive(0, 0, -strafePower, true, false);
                                } else if (autoSubTimer.get() > driveStrafeTime + moveForwardTime && autoSubTimer
                                                .get() < secondPushBackTime + driveStrafeTime + moveForwardTime) {
                                        m_Drive.drive(0, -drivePushPower, 0, true, false);
                                } else if (autoSubTimer.get() > secondPushBackTime + driveStrafeTime + moveForwardTime
                                                && autoSubTimer.get() < secondPullForwardtime + secondPushBackTime
                                                                + driveStrafeTime + moveForwardTime) {
                                        // KK 03/01
                                        // m_Drive.drive(0, -driveResetPower, 0, true, false);
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
                                        m_Drive.move((double) 90, (double) .5, (double) 70, true);
                       
                                        currentState = AutoEnumeration.End;
                                } else {
                                        double drivePower = 0.6;
                                        drivePower *= shouldDriveForward ? -1 : 1;
                                        m_Drive.drive(0, drivePower, 0, true, false);
                                }
                        }
                        break;
                case AutoEnumeration.End:
                        m_Shooter.shooterKill();
                        shouldRunShooter = false;
                        m_Conveyor.conveyorKill();
                        m_Drive.drive(0, 0, 0, true, false);
                        break;
                default:
                        DriverStation.reportError("Hit default auto case", false);
                        shouldRunShooter = false;
                        m_Shooter.shooterKill();
                        m_Conveyor.conveyorKill();
                        m_Drive.drive(0, 0, 0, true, false);
                        break;
                }
        }
        @Override
        public void teleopInit() {
            m_LimeLight.turnLEDOn();
            m_IntakeDrop.set(true);
            m_Drive.SetDriverMode();
        }
        /**
         * This function is called periodically during operator control.
         */
        @Override
        public void teleopPeriodic() {

                boolean gamepadY = false;
                boolean gamepadX = false;
                boolean gamepadA = false;
                boolean gamepadB = false;
                boolean gamepadRightBumper = false;
                boolean gamepadLeftBumper = false;
                boolean gamepadStart = false;
                double gamepadLeftTrigger = 0.0;
                boolean gamepadLeftTriggerPulled = false;
                double gamepadRightTrigger = 0.0;
                boolean gamepadRightTriggerPulled = false;
                
                double gamepadRightJoystickY = 0.0;
                double gamepadLeftJoystickY = 0.0;
                int gamepadPOV = 0;

                
                gamepadPOV=m_gamepad.getPOV(0);
                gamepadY = m_gamepad.getRawButton(4);
                gamepadX = m_gamepad.getRawButton(3);
                gamepadA = m_gamepad.getRawButton(1);
                gamepadB = m_gamepad.getRawButton(2);
                gamepadStart = m_gamepad.getRawButton(8);
                gamepadRightBumper = m_gamepad.getRawButton(6);
                gamepadLeftBumper = m_gamepad.getRawButton(5);
                gamepadLeftTrigger = m_gamepad.getRawAxis(2);
                gamepadLeftTriggerPulled = (gamepadLeftTrigger > 0.3);
                gamepadRightTrigger = m_gamepad.getRawAxis(3);
                gamepadRightTriggerPulled = (gamepadRightTrigger > 0.3);
                gamepadRightJoystickY = m_gamepad.getRawAxis(5);
                gamepadLeftJoystickY = m_gamepad.getRawAxis(1);

                //kk 04/05/20 variable long shooting speed
                if( m_joystickRight.getRawButton(5)){
                        m_LongShotSpeed=.633;
                }else if(m_joystickRight.getRawButton(6)){
                        m_LongShotSpeed=.650;
                }else if(m_joystickRight.getRawButton(7)){
                        m_LongShotSpeed=.680;
                }else if(m_joystickRight.getRawButton(8)){
                        m_LongShotSpeed=.690;
                }else if(m_joystickRight.getRawButton(9)){
                        m_LongShotSpeed=.700;
                
                }
                //KK 07/09/20
                m_LongShotSpeed=networkTableEntryShooterPower.getNumber(1.0).doubleValue();
                //KK 07/10/20 shoot from InitLine
                m_LongShotSpeed=Constants.kAutoShootSpeed;
                if(m_joystickRight.getRawButton(10)|| gamepadPOV==0){
                        //start shooter
                      
                        m_Drive.targetAlign(m_LimeLight,Constants.kInitLineShootingDistance,Constants.LimeLightValues.targetXPosShoot,false,false);
                       
                }
                if(m_joystickRight.getRawButton(11)|| gamepadPOV==90){
                        m_Drive.targetAlign(m_LimeLight,Constants.kSafeZoneShootingDistance,Constants.LimeLightValues.targetXPosSafeZone,false,false);
                }
                
                if(gamepadPOV==180){
                        m_Drive.targetTurn(m_LimeLight);
                }
                SmartDashboard.putBoolean("is Ready to shoot", m_Drive.isReadyToShoot());
        
                isShootingLong = m_joystickRight.getRawButton(1) || gamepadRightTriggerPulled || m_Drive.isAutoShootLong();
                isShootingShort = m_joystickLeft.getRawButton(2) || gamepadLeftTriggerPulled;

                if (gamepadStart) {
                
                        m_ahrs.calibrate();
                        m_ahrs.zeroYaw();
                        ;
                }
                if (gamepadA) {
                        m_IntakeDrop.set(true);
                }

                // #region Swerve Drive Code
                try {
                        // Get gryo angle
                        gyroAngle = m_ahrs.getFusedHeading();
                        // fieldCentric is true only if you can recieve a gyro angle
                        fieldCentric = true;
                } catch (final Exception e) {
                        DriverStation.reportError("Couldn't recieve gyro angle:" + e, true);
                        DriverStation.reportError("Moving to otCentric Controls", true);
                        gyroAngle = 0;
                }

                // Run Drive
                // if you changs
                final double strafe = m_joystickLeft.getRawAxis(0);
                final double forward = m_joystickLeft.getRawAxis(1);
                final double rotateClockwise = -m_joystickRight.getRawAxis(0);

                m_Drive.drive(rotateClockwise, forward, strafe, fieldCentric, false);
              /*commenting dead loop
                if (m_joystickLeft.getRawButton(8)) {
                        while (shooterSonarSensor.getValue() > 2000) {

                        }

                }
                */
                // if (m_joystickLeft.getRawButton(9)){
                // //Joe's Auto
                // m_Drive.move((double)0,(double).2,(double)172,false);
                // m_Drive.move((double)-45,(double).2,(double)48,false);
                // m_Drive.move((double)-90,(double).2,(double)36,false);
                // m_Drive.move((double)-135,(double).2,(double)54,false);
                // m_Drive.move((double)45,(double).2,(double)36,false);
                // m_Drive.move((double)45,(double).2,(double)96,false); }
                // m_Drive.move((double)0,(double).2,(double)48,false);
                // m_Drive.move((double)45,(double).2,(double)36,false);
                // }

                // DriverStation.reportError(rotateClockwise + "", false);

                // shuffleboardGyroCompass.setDouble(gyroAngle);

                m_ledStrip.setMode(LEDStrip.MODE_WAVE);
                LED_Color = Colors.ORANGE;
                m_ShooterLights.set(off);

                // Vision.setDriverMode(!isShooting);

                if (isShootingLong) {
                        m_ledStrip.setMode(LEDStrip.MODE_BAR);
                        m_ledStrip.setPercentage(Vision.getLedValue());
                        m_ShooterLights.set(on);

                        // Shooter Motor
                        //m_Shooter.voltageRun(Constants.kShooting20);
                        m_Shooter.voltageRun(m_LongShotSpeed);
                        m_LongShooterSolenoid.set(LongUp); // this is actually up
                        m_ShortShooterSolenoid.set(ShortDown);
                       
                } else if (isShootingShort) {
                        // Leds
                        m_ledStrip.setMode(LEDStrip.MODE_SOLID);
                        LED_Color = Colors.GREEN;

                        // Shooter Motor
                        m_Shooter.voltageRun(Constants.kShootingUpClose);

                        // Pneumatics
                        if (!shooterPneumaticTimerSet) {
                                shooterPneumaticTimer.reset();
                                shooterPneumaticTimer.start();
                                shooterPneumaticTimerSet = true;
                        }
                        if (shooterPneumaticTimer.get() < 0.5) {
                                // m_Drive.drive(rotateClockwise, 0.0, strafe, fieldCentric, false);
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

                        // 5/29/21 3:14pm
                        // m_LongShooterSolenoid.set(LongUp);
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
                                m_ShooterLights.set(on);

                                // Shooter Motor
                                m_Shooter.shooterKill();

                                // Pneumatics
                                m_LongShooterSolenoid.set(LongDown);
                                m_ShortShooterSolenoid.set(ShortDown);
                        }

                }

                m_ledStrip.setColor(LED_Color);
                
                // if (isShootingLong && Vision.getIsValid() && false) {
                // m_Drive.drive(Vision.getTurnValue(), forward, strafe, fieldCentric,
                // isShootingLong);
                // }
                // else if (m_joystickLeft.getRawButton(1)) {
                // m_Drive.drive(Vision.getBallTurnValue(), 0, 0, fieldCentric, true);
                // }
                // else {
                // m_Drive.drive(rotateClockwise, forward, strafe, fieldCentric, false);
                // }

                // using a Gyro Widget on the Shuffleboard, have to convert our angle 180 ->
                // -180 to 0 - 360
                // then put the converted angle into the Gyro Adapter widget.
                // frontLeftGyroAdapter.setAngle(m_Drive.ShuffleBoardAngleConversion(m_Drive.frontLeftAngle));
                // frontRightGyroAdapter.setAngle(m_Drive.ShuffleBoardAngleConversion(m_Drive.frontRightAngle));
                // backLeftGyroAdapter.setAngle(m_Drive.ShuffleBoardAngleConversion(m_Drive.backLeftAngle));
                // backRightGyroAdapter.setAngle(m_Drive.ShuffleBoardAngleConversion(m_Drive.backRightAngle));

                // networkTableEntryFrontLeftSpeed.setDouble(m_Drive.frontLeftSpeed);
                // networkTableEntryFrontRightSpeed.setDouble(m_Drive.frontRightSpeed);
                // networkTableEntryBackLeftSpeed.setDouble(m_Drive.backLeftSpeed);
                // networkTableEntryBackRightSpeed.setDouble(m_Drive.backRightSpeed);
                // #endregion

                // #region Subsystem Controls

                // #region Intake control

                if (m_joystickRight.getRawButton(3) || gamepadX) {
                        m_Intake.intake(1.0); //KK was .75 
                        m_Intake.lower();
                } else if (m_joystickRight.getRawButton(4) ) {
                        m_Intake.intake(.5);
                        m_Intake.lower();
                } else if (m_joystickLeft.getRawButton(3) || gamepadB) {
                        m_Intake.outtake(1.0);
                        m_Intake.lower();
                } else if (m_joystickRight.getRawButton(9) || gamepadY || m_joystickLeft.getRawButton(4)
                                || m_joystickLeft.getRawButton(5)) {
                        m_Intake.raise();
                } else {
                        m_Intake.intakeKill();
                }
                // #endregion

                // #region Conveyor control

                conveyorSensorTriggered = false;
                conveyorShutdownTriggered = false;

                // if (conveyorSensorTriggered) {
                // conveyorIsLoaded = true;
                // }
              
                if (m_joystickRight.getRawButton(2) || gamepadRightBumper ) {
                        m_Conveyor.feedShooter();
                } else if (m_joystickLeft.getRawButton(7) || gamepadLeftBumper) {
                        m_Conveyor.runReverse();
                }
                // else if (!conveyorButtonHasBeenPressed) {
                // if (conveyorShutdownTriggered) {
                // // m_Intake.intakeKill();
                // m_Conveyor.conveyorKill();
                // } else {
                // if (conveyorSensorTriggered) {
                // // m_Intake.intake(.5);
                // m_Conveyor.loadConveyor();
                // conveyorIsLoaded = true;
                // } else {
                // if (conveyorTimerHasBeenSet) {
                // if (conveyorDelayTimer.hasPeriodPassed(0.2)) {
                // m_Conveyor.conveyorKill();
                // conveyorTimerHasBeenSet = false;
                // conveyorIsLoaded = false;
                // } else {
                // m_Conveyor.loadConveyor();
                // }
                // } else if (conveyorIsLoaded) {
                // conveyorDelayTimer.reset();
                // conveyorDelayTimer.start();
                // sconveyorTimerHasBeenSet = true;
                // }
                // // m_Intake.intakeKill();
                // }
                // }
                // }
                else {
                        m_Conveyor.conveyorKill();
                }
                // #endregion

                // #region Climber control
                climbPullPower = gamepadLeftJoystickY;
                climbRaisePower = gamepadRightJoystickY;

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
        //This funcion is caused when a robot is disableds
        public void disabledPeriodic(){
                m_LimeLight.turnLEDOff();
        }
        @Override
        public void disabledInit() {

        }
}
