// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Map;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.Constants.AutoModes;
import frc.robot.Constants.ButtonConstants;
import frc.robot.Constants.InputControllers;
import frc.robot.classes.Limelight;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.networktables.NetworkTableEntry;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.smartdashboard.*;
/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // The robot's subsystems and commands are defined here...
    private AHRS m_ahrs = new AHRS(SPI.Port.kMXP);
    private final DriveSubsystem m_RobotDrive = new DriveSubsystem(m_ahrs);
    // private final SwerveSubsystem m_RobotDrive = new SwerveSubsystem();
    private final Limelight m_Limelight = new Limelight();

    // Shooter(Integer BottomMotorPort, Integer TopMotorPort)
    private final ShooterSubsystem m_Shooter = new ShooterSubsystem(25, 26);
    private final ConveyorSubsystem m_Conveyor = new ConveyorSubsystem(Constants.ConveyorPort);
    private final IntakeSubsystem m_Intake = new IntakeSubsystem(Constants.IntakePort);
    // #region Shuffleboard

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

    // private final ExampleCommand m_autoCommand = new
    // ExampleCommand(m_exampleSubsystem);

    private XboxController m_xBoxDriver = new XboxController(InputControllers.kXboxDrive);
    // private XboxController m_xBoxOperator = new
    // XboxController(InputControllers.kXboxOperator);

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // Configure the button bindings
        configureButtonBindings();

        // Default Drive
        // TODO: This mapping works for our drive, might not for WPI swerve
        // m_RobotDrive.setDefaultCommand(
        // new DefaultDriveCommand(m_RobotDrive,
        // () -> -m_xBoxDriver.getRightX(),
        // () -> m_xBoxDriver.getLeftY(),
        // () -> m_xBoxDriver.getLeftX()));
        m_RobotDrive.setDefaultCommand(
                new DefaultDriveCommand(m_RobotDrive,
                        () -> -m_xBoxDriver.getRightX(),
                        () -> m_xBoxDriver.getLeftY(),
                        () -> m_xBoxDriver.getLeftX()));

    }
    
    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
     * it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {

        final ParallelCommandGroup aimAndSpinCommand = new ParallelCommandGroup(new ShooterCommand(m_Shooter,m_Limelight,AutoModes.AutoShotTopSpeed,AutoModes.AutoShotBottomSpeed,false),
        new DefaultDriveCommand(m_RobotDrive,
                () -> m_Limelight.getRotationPower(m_Limelight.getX(), 15.0 * m_xBoxDriver.getLeftX()),
                () -> m_xBoxDriver.getLeftY(),
                () -> m_xBoxDriver.getLeftX()));

      
        final ConveyorCommand conveyorUpCommand = new ConveyorCommand(m_Conveyor,Constants.conveyorUpSpeed);
        final ConveyorCommand conveyorDownCommand = new ConveyorCommand(m_Conveyor,Constants.conveyorUpSpeed);
        final IntakeCommand intakeInCommand = new IntakeCommand(m_Intake,Constants.intakeSpeed);
        final IntakeCommand intakeOutCommand = new IntakeCommand(m_Intake,-Constants.intakeSpeed);
        final ShooterCommand shootCommand = new ShooterCommand(m_Shooter,m_Limelight,AutoModes.AutoShotTopSpeed,AutoModes.AutoShotBottomSpeed,false);
        // Creates a new JoystickButton object for button 1 (xBox A) on m_RobotDrive
        JoystickButton aimAndSpin = new JoystickButton(m_xBoxDriver, ButtonConstants.AimandShoot);
        SmartDashboard.putString("Aim and Spin ","Left-Button " + ButtonConstants.AimandShoot);
      
        JoystickButton manualShoot = new JoystickButton(m_xBoxDriver, ButtonConstants.ManualShoot);
        SmartDashboard.putString("Manual Shoot","Left-Button " + ButtonConstants.ManualShoot);
      
        // Creates a new JoystickButton object for button 6 (xBox rightBumper) on
        // m_RobotDrive
        JoystickButton conveyorUpButton = new JoystickButton(m_xBoxDriver, ButtonConstants.ConveyerIn);
        SmartDashboard.putString("Conveyer Up","Left-Button " + ButtonConstants.ConveyerIn);
      
        // Creates a new JoystickButton object for button 2 (xBox B) on m_RobotDrive
        JoystickButton conveyorDownButton = new JoystickButton(m_xBoxDriver, ButtonConstants.ConveyerOut);
        SmartDashboard.putString("Shooting","Left-Button " + ButtonConstants.ConveyerOut);

        JoystickButton intakeInButton= new JoystickButton(m_xBoxDriver, ButtonConstants.IntakeIn);
        SmartDashboard.putString("Intake In","Left-Button " + ButtonConstants.IntakeIn);

        JoystickButton intakeOutButton = new JoystickButton(m_xBoxDriver, ButtonConstants.IntakeOut);
        SmartDashboard.putString("Intake Out","Left-Button " + ButtonConstants.IntakeOut);

        aimAndSpin.whenHeld(aimAndSpinCommand);
        manualShoot.whenHeld(shootCommand);
        conveyorUpButton.whenHeld(conveyorUpCommand);
        conveyorDownButton.whenHeld(conveyorDownCommand);
        intakeOutButton.whenHeld(intakeOutCommand);
        intakeInButton.whenHeld(intakeInCommand);    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand(Integer selectedMode) {
        Command autoCommand = new AutoDoNothingCommand(); // Default Command is DoNothing
        System.out.println("Autonomouse Selected Mode = " + selectedMode);
        switch (selectedMode) {
          case AutoModes.autoMoveForward:
             autoCommand= new AutoMoveCommand(m_RobotDrive,0);
            break;
          case AutoModes.autoMoveShoot:
            autoCommand=new SequentialCommandGroup(
                    new zSpinLoadShootCommand(m_Shooter, m_Conveyor, AutoModes.AutoShotTopSpeed, AutoModes.AutoShotBottomSpeed),
                    new AutoMoveCommand(m_RobotDrive,0));

            // autoCommand= new AutoMoveCommand(m_RobotDrive,-AutoConstants.moveSpeed,2);
            break;
          case AutoModes.autoMoveBackwardsOutake:
            // autoCommand= new AutoMoveCommand(m_RobotDrive,-AutoConstants.moveSpeed,2);
            break;
          case AutoModes.autoMoveBackwardsShot:
          
                  autoCommand=new SequentialCommandGroup(
                new zSpinLoadShootCommand(m_Shooter, m_Conveyor, AutoModes.AutoShotTopSpeed, AutoModes.AutoShotBottomSpeed),
                new AutoMoveCommand(m_RobotDrive,180));

            break;
          case AutoModes.autoMoveShootMoveGrab:
            // autoCommand= new AutoMoveCommand(m_RobotDrive,-AutoConstants.moveSpeed,2);
            break;
          case AutoModes.autoMoveShootMoveGrabShot1:
            // autoCommand= new AutoMoveCommand(m_RobotDrive,-AutoConstants.moveSpeed,2);
            break;
          case AutoModes.autoMoveShotMoveGrabMoveLeftGrabShot2:
            // autoCommand= new AutoMoveCommand(m_RobotDrive,-AutoConstants.moveSpeed,2);
            break;
          case AutoModes.autoMoveShotMoveGrabMoveRightGrabShot2:
            // autoCommand= new AutoMoveCommand(m_RobotDrive,-AutoConstants.moveSpeed,2);
            break;
          case AutoModes.autoMoveGrabTrackRightShoot:
            // autoCommand= new AutoMoveCommand(m_RobotDrive,-AutoConstants.moveSpeed,2);
            break;
          case AutoModes.autoMoveGrabTrackLeftShoot:
            // autoCommand= new AutoMoveCommand(m_RobotDrive,-AutoConstants.moveSpeed,2);
            break;        
          default:
            autoCommand = new AutoDoNothingCommand();
        }
        return autoCommand;
        // return m_autoCommand;
    }
}
