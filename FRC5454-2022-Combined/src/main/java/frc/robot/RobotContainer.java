package frc.robot;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.commands.DriveCommand;
import frc.robot.Constants.AutoModes;
import frc.robot.Constants.ButtonConstants;
import frc.robot.Constants.InputControllers;
import frc.robot.classes.Limelight;
import frc.robot.classes.SpectrumAxisButton;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.util.AutonomousChooser;
import frc.robot.util.AutonomousTrajectories;
import edu.wpi.first.wpilibj.XboxController;

import java.io.IOException;
import java.util.Map;

public class RobotContainer {
    private XboxController m_xBoxDriver = new XboxController(InputControllers.kXboxDrive);
    private XboxController m_xBoxOperator = new XboxController(InputControllers.kXboxOperator);

    private final DrivetrainSubsystem drivetrainSubsystem = new DrivetrainSubsystem();
    private final Limelight m_Limelight = new Limelight(Constants.LimeLightValues.targetHeight,
            Constants.LimeLightValues.limelightHeight, Constants.LimeLightValues.limelightAngle,
            Constants.LimeLightValues.kVisionXOffset);

    // Shooter(Integer BottomMotorPort, Integer TopMotorPort)
    private final ShooterSubsystem m_Shooter = new ShooterSubsystem(Constants.TopShooterPort,
            Constants.BottomShooterPort);
    private final ConveyorSubsystem m_Conveyor = new ConveyorSubsystem(Constants.ConveyorPort);
    private final FeederSubsystem m_Feeder = new FeederSubsystem(Constants.FeederPort);
    private final IntakeSubsystem m_Intake = new IntakeSubsystem(Constants.IntakePort);
    private final IntakeSubsystem m_IntakeInner = new IntakeSubsystem(Constants.IntakeInnerPort);

    private final ClimbSubsystem m_Climb = new ClimbSubsystem(Constants.ClimberPort,
            Constants.LimitSwitches.ClimberBottom, Constants.LimitSwitches.ClimberTop);
    private final PneumaticsSubsystem m_Pnuematics = new PneumaticsSubsystem(Constants.Pneumatics.CompressorID);
    private final TurretSubsystem m_turret = new TurretSubsystem(Constants.TurretPort,
            Constants.LimitSwitches.TurretLeft,
            Constants.LimitSwitches.TurretRight,
            Constants.turretHomePos,
            Constants.turretHomeSpeed);

    private final PowerDistribution m_robotPDH = new PowerDistribution(1, PowerDistribution.ModuleType.kRev);

    // #region Create Shuffleboard Tabs
    private static ShuffleboardTab SwerveTab = Shuffleboard.getTab("Swerve");
    private static ShuffleboardTab SwerveEncoders = Shuffleboard.getTab("SwerveEncoders");
    private static ShuffleboardTab ShooterTab = Shuffleboard.getTab("Shooter");
    private static ShuffleboardTab ControlTab = Shuffleboard.getTab("Controls");
    private static ShuffleboardTab PDPTab = Shuffleboard.getTab("PDP");
    // #endregion

    // #region NetworkEntries
    // Create Network Table Entries

    static NetworkTableEntry networkTableEntryVisionDistance = ShooterTab.add("Vision Distance", 0)
            .withWidget(BuiltInWidgets.kNumberBar).withSize(2, 2).getEntry();

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

    static NetworkTableEntry shuffleboardGyroFused = SwerveTab.add("Gyro - Fused Heading", 0)
            .withWidget(BuiltInWidgets.kTextView).getEntry();

    static NetworkTableEntry shuffleboardDrive = ControlTab.add("Drive Control", "Left Stick").getEntry();
    static NetworkTableEntry shuffleboarRotate = ControlTab.add("Swerve Control", "Right Stick").getEntry();
    static NetworkTableEntry shuffleboardBallFeedUp = ControlTab.add("Feed Up", "").getEntry();
    static NetworkTableEntry shuffleboardBallFeedDown = ControlTab.add("Feed Down", "").getEntry();
    static NetworkTableEntry shuffleboardIntakeInD = ControlTab.add("D-Intake In", "").getEntry();
    static NetworkTableEntry shuffleboardIntakeOutD = ControlTab.add("D-Intake Out", "").getEntry();
    static NetworkTableEntry shuffleboardIntakeInO = ControlTab.add("O-Intake In", "").getEntry();
    static NetworkTableEntry shuffleboardIntakeOutO = ControlTab.add("O-Intake Out", "").getEntry();
    static NetworkTableEntry shuffleboardIntakeArmD = ControlTab.add("D-Intake Arm", "").getEntry();
    static NetworkTableEntry shuffleboardIntakeArmO = ControlTab.add("O-Intake Arm", "").getEntry();

    static NetworkTableEntry shuffleboardGyroResetD = ControlTab.add("D-Gyro Reset", "").getEntry();
    static NetworkTableEntry shuffleboardGyroResetO = ControlTab.add("O-Gyro Reset", "").getEntry();
    static NetworkTableEntry shuffleboardAutoShootD = ControlTab.add("D-Auto Shoot", "").getEntry();
    static NetworkTableEntry shuffleboardAutoShootO = ControlTab.add("O-Auto Shoot", "").getEntry();

    static NetworkTableEntry shuffleboardTurretTurn = ControlTab.add("Turret Turn", "").getEntry();
    static NetworkTableEntry shuffleboardClimbLift = ControlTab.add("Climb Lift", "").getEntry();
    static NetworkTableEntry shuffleboardClimbLower = ControlTab.add("Climb Down", "").getEntry();

    static NetworkTableEntry shuffleboardOperatorPivotArm = ControlTab.add("Pivot Arms", "").getEntry();
    static NetworkTableEntry shuffleboardOperatorManualShoot = ControlTab.add("Manual Shoot", "").getEntry();

    static NetworkTableEntry shuffleboardPDPStickyCANFaults = PDPTab.add("Sticky CAN Faults", "").getEntry();
    static NetworkTableEntry shuffleboardPDPTotalCurrent = PDPTab.add("Total Current", "").getEntry();
    static NetworkTableEntry shuffleboardPDPCurrentC0 = PDPTab.add("Current C0", "").getEntry();
    static NetworkTableEntry shuffleboardPDPCurrentC1 = PDPTab.add("Current C1", "").getEntry();
    static NetworkTableEntry shuffleboardPDPCurrentC2 = PDPTab.add("Current C2", "").getEntry();
    static NetworkTableEntry shuffleboardPDPCurrentC3 = PDPTab.add("Current C3", "").getEntry();
    static NetworkTableEntry shuffleboardPDPCurrentC4 = PDPTab.add("Current C4", "").getEntry();
    static NetworkTableEntry shuffleboardPDPCurrentC5 = PDPTab.add("Current C5", "").getEntry();
    static NetworkTableEntry shuffleboardPDPCurrentC6 = PDPTab.add("Current C6", "").getEntry();
    static NetworkTableEntry shuffleboardPDPCurrentC7 = PDPTab.add("Current C7", "").getEntry();
    static NetworkTableEntry shuffleboardPDPCurrentC8 = PDPTab.add("Current C8", "").getEntry();
    static NetworkTableEntry shuffleboardPDPCurrentC9 = PDPTab.add("Current C9", "").getEntry();
    static NetworkTableEntry shuffleboardPDPCurrentC10 = PDPTab.add("Current C10", "").getEntry();
    static NetworkTableEntry shuffleboardPDPCurrentC11 = PDPTab.add("Current C11", "").getEntry();
    static NetworkTableEntry shuffleboardPDPCurrentC12 = PDPTab.add("Current C12", "").getEntry();
    static NetworkTableEntry shuffleboardPDPCurrentC13 = PDPTab.add("Current C13", "").getEntry();
    static NetworkTableEntry shuffleboardPDPCurrentC14 = PDPTab.add("Current C14", "").getEntry();
    static NetworkTableEntry shuffleboardPDPCurrentC15 = PDPTab.add("Current C15", "").getEntry();
    static NetworkTableEntry shuffleboardPDPCurrentC16 = PDPTab.add("Current C16", "").getEntry();
    static NetworkTableEntry shuffleboardPDPCurrentC17 = PDPTab.add("Current C17", "").getEntry();
    static NetworkTableEntry shuffleboardPDPCurrentC18 = PDPTab.add("Current C18", "").getEntry();
    static NetworkTableEntry shuffleboardPDPCurrentC19 = PDPTab.add("Current C19", "").getEntry();
    static NetworkTableEntry shuffleboardPDPCurrentC20 = PDPTab.add("Current C20", "").getEntry();
    static NetworkTableEntry shuffleboardPDPCurrentC21 = PDPTab.add("Current C21", "").getEntry();
    static NetworkTableEntry shuffleboardPDPCurrentC22 = PDPTab.add("Current C22", "").getEntry();
    static NetworkTableEntry shuffleboardPDPCurrentC23 = PDPTab.add("Current C23", "").getEntry();

    static NetworkTableEntry shuffleboardTurretPos = ShooterTab.add("Turret Position", "").getEntry();

    static NetworkTableEntry shuffleboardLeftLimit = ShooterTab.add("Left Limit", "").getEntry();
    static NetworkTableEntry shuffleboardRightLimit = ShooterTab.add("Right Limit", "").getEntry();
    static NetworkTableEntry shuffleboardShooterTop = ShooterTab.add("Top Speed", Constants.AutoModes.AutoShotTopSpeed)
            .getEntry();
    static NetworkTableEntry shuffleboardShooterBottom = ShooterTab
            .add("Bottom Speed", Constants.AutoModes.AutoShotBottomSpeed).getEntry();

    static NetworkTableEntry shuffleboardShooterTopVel = ShooterTab.add("Top Velocity", "").getEntry();
    static NetworkTableEntry shuffleboardShooterBottomVel = ShooterTab.add("Bottom Velocity", "").getEntry();
    static NetworkTableEntry shuffleobardShooterMultipler = ShooterTab.add("Shooter Adjustment", 1.0)
            .withWidget(BuiltInWidgets.kNumberSlider)
            .withProperties(Map.of("min", -1, "max", 5))
            .getEntry();

    static String ShuffleboardLogString;
    // #endregion
    // #endregion

    private AutonomousTrajectories autonomousTrajectories;
    private final AutonomousChooser autonomousChooser;

    private boolean m_turretHasReset = false;

    public RobotContainer() {
        try {
            autonomousTrajectories = new AutonomousTrajectories(DrivetrainSubsystem.TRAJECTORY_CONSTRAINTS);
        }
        catch (IOException e) {
            e.printStackTrace();
        }
        autonomousChooser = new AutonomousChooser(autonomousTrajectories);
        ;

        CommandScheduler.getInstance().setDefaultCommand(drivetrainSubsystem,
                new DriveCommand(drivetrainSubsystem, () -> getDriveForwardAxis(), () -> getDriveStrafeAxis(),
                        () -> getDriveRotationAxis()));

        // force default shoot multiplier
        shuffleobardShooterMultipler.setDouble(1.0);
        // Configure the button bindings
        configureButtonBindings();
    }

    private void configureButtonBindings() {
        // FIXIt when done getting shooter values
        double topSpeed = shuffleboardShooterTop.getDouble(0);
        double bottomSpeed = shuffleboardShooterBottom.getDouble(0);

        final ParallelCommandGroup LoadandShootCommand = new ParallelCommandGroup(
                new ShooterCommand(m_Shooter, m_Limelight, topSpeed, bottomSpeed, true),
                new ConveyorCommand(m_Conveyor, Constants.conveyorUpSpeed),
                new FeederCommand(m_Feeder, Constants.FeederSpeed));

        final ParallelCommandGroup ManualShootCommand = new ParallelCommandGroup(
                new ShooterCommand(m_Shooter, m_Limelight, Constants.ManualShots.Shot1Top,
                        Constants.ManualShots.Shot1Bottom,
                        false),
                new ConveyorCommand(m_Conveyor, Constants.conveyorUpSpeed),
                new FeederCommand(m_Feeder, Constants.FeederSpeed));

        final SequentialCommandGroup releaseAndResetClimb = new SequentialCommandGroup(
                new HookCablesReleaseCommand(m_Pnuematics),
                new ClimbArmResetCommand(m_Pnuematics));

        final ParallelCommandGroup ManualShooter1Command = new ParallelCommandGroup(
                new ShooterCommand(m_Shooter, m_Limelight, Constants.ManualShots.Shot1Top,
                        Constants.ManualShots.Shot1Bottom,
                        false),
                new ConveyorCommand(m_Conveyor, Constants.conveyorUpSpeed),
                new FeederCommand(m_Feeder, Constants.FeederSpeed));

        final ParallelCommandGroup ManualShooter2Command = new ParallelCommandGroup(
                new ShooterCommand(m_Shooter, m_Limelight, Constants.ManualShots.Shot2Top,
                        Constants.ManualShots.Shot2Bottom,
                        false),
                new ConveyorCommand(m_Conveyor, Constants.conveyorUpSpeed),
                new FeederCommand(m_Feeder, Constants.FeederSpeed));

        final ParallelCommandGroup ManualShooter3Command = new ParallelCommandGroup(
                new ShooterCommand(m_Shooter, m_Limelight, Constants.ManualShots.Shot3Top,
                        Constants.ManualShots.Shot3Bottom,
                        false),
                new ConveyorCommand(m_Conveyor, Constants.conveyorUpSpeed),
                new FeederCommand(m_Feeder, Constants.FeederSpeed));

        final ParallelCommandGroup ManualShooter4Command = new ParallelCommandGroup(
                new ShooterCommand(m_Shooter, m_Limelight, Constants.ManualShots.Shot4Top,
                        Constants.ManualShots.Shot4Bottom,
                        false),
                new ConveyorCommand(m_Conveyor, Constants.conveyorUpSpeed),
                new FeederCommand(m_Feeder, Constants.FeederSpeed));

        // final ConveyorCommand conveyorUpCommand = new
        // ConveyorCommand(m_Conveyor,Constants.conveyorUpSpeed);

        // final ConveyorCommand conveyorDownCommand = new
        // ConveyorCommand(m_Conveyor,Constants.conveyorDownSpeed);

        final ParallelCommandGroup conveyorUpCommand = new ParallelCommandGroup(
                new ConveyorCommand(m_Conveyor, Constants.conveyorUpSpeed),
                new IntakeCommand(m_IntakeInner, Constants.intakeInnerSpeed));

        final ParallelCommandGroup conveyorDownCommand = new ParallelCommandGroup(
                new ConveyorCommand(m_Conveyor, Constants.conveyorDownSpeed),
                new IntakeCommand(m_IntakeInner, -Constants.intakeInnerSpeed));

        // final IntakeCommand intakeInCommand = new
        // IntakeCommand(m_Intake,Constants.intakeSpeed);

        // final IntakeCommand intakeOutCommand = new
        // IntakeCommand(m_Intake,-Constants.intakeSpeed);

        final zIntakeConveyCommand intakeInCommand = new zIntakeConveyCommand(m_Intake, m_IntakeInner,
                Constants.intakeSpeed, Constants.intakeInnerSpeed, m_Conveyor,Constants.conveyorUpSpeed, m_Feeder, -Constants.FeederSpeed);

        final zIntakeConveyCommand intakeOutCommand = new zIntakeConveyCommand(m_Intake, m_IntakeInner,
                -Constants.intakeSpeed,-Constants.intakeInnerSpeed, m_Conveyor, Constants.conveyorDownSpeed, m_Feeder, -Constants.FeederSpeed);

        final ShooterCommand shootCommand = new ShooterCommand(m_Shooter, m_Limelight, AutoModes.AutoShotTopSpeed,
                AutoModes.AutoShotBottomSpeed, false);

        final FeederCommand feedUpCommand = new FeederCommand(m_Feeder, Constants.FeederSpeed);

        final ClimbCommand climbUpCommand = new ClimbCommand(m_Climb, m_Pnuematics, m_turret, Constants.climbUpSpeed);

        final ClimbCommand climbDownCommand = new ClimbCommand(m_Climb, m_Pnuematics, m_turret,
                Constants.climbDownSpeed);

        final IntakeArmCommand intakeArmCommand = new IntakeArmCommand(m_Pnuematics);

        final ClimbArmCommand climbArmsCommand = new ClimbArmCommand(m_Pnuematics);

        final HookCablesCommand climbHooksCommand = new HookCablesCommand(m_Pnuematics);

        final TurretCommand turretLeftCommand = new TurretCommand(m_turret, Constants.turretSpeed);

        final TurretCommand turretRightCommand = new TurretCommand(m_turret, -Constants.turretSpeed);

        final TurretCommand turretStopCommand = new TurretCommand(m_turret, 0);

        final GyroResetCommand gyroResetCommand = new GyroResetCommand(getDrivetrainSubsystem(), m_Limelight);

        final zTurretLimelightCommand turretAutoCommand = new zTurretLimelightCommand(m_turret, m_Limelight,
                Constants.turretSpeed, Constants.turretMinSpeed, Constants.LimeLightValues.targetXPosRange);

        // final LatchCommand latchCommand = new LatchCommand(m_Pnuematics);

        JoystickButton driverAutoShoot = new JoystickButton(m_xBoxDriver, ButtonConstants.DriverAutoShoot);
        shuffleboardAutoShootD.setString("D-Button " + ButtonConstants.DriverAutoShoot);

        SpectrumAxisButton operatorAutoShoot = new SpectrumAxisButton(m_xBoxOperator,
                ButtonConstants.OperatorAutoShootAxis,
                ButtonConstants.TriggerThreshold, SpectrumAxisButton.ThresholdType.GREATER_THAN);
        shuffleboardAutoShootO.setString("Right Trigger");

        JoystickButton driverIntakeIn = new JoystickButton(m_xBoxDriver, ButtonConstants.DriverIntakeIn);
        shuffleboardIntakeInD.setString("D-Button " + ButtonConstants.DriverIntakeIn);

        JoystickButton operatorIntakeIn = new JoystickButton(m_xBoxOperator, ButtonConstants.OperatorIntakeIn);
        shuffleboardIntakeInO.setString("O-Button " + ButtonConstants.OperatorIntakeIn);

        JoystickButton driverIntakeOut = new JoystickButton(m_xBoxDriver, ButtonConstants.DriverIntakeOut);
        shuffleboardIntakeOutD.setString("D-Button " + ButtonConstants.DriverIntakeOut);

        JoystickButton operatorIntakeOut = new JoystickButton(m_xBoxOperator, ButtonConstants.OperatorIntakeOut);
        shuffleboardIntakeOutO.setString("O-Button " + ButtonConstants.OperatorIntakeOut);

        JoystickButton driverIntakeArm = new JoystickButton(m_xBoxDriver, ButtonConstants.DriverIntakeArm);
        shuffleboardIntakeArmD.setString("D-Button" + ButtonConstants.DriverIntakeArm);
        JoystickButton operatorIntakeArm = new JoystickButton(m_xBoxOperator, ButtonConstants.OperatorIntakeArm);
        shuffleboardIntakeArmO.setString("O-Button" + ButtonConstants.OperatorIntakeArm);

        JoystickButton driverGyroReset = new JoystickButton(m_xBoxDriver, ButtonConstants.DriverGyroReset);
        shuffleboardGyroResetD.setString("D-Button " + Constants.ButtonConstants.DriverGyroReset);
        JoystickButton driverGyroReset2 = new JoystickButton(m_xBoxDriver, ButtonConstants.DriverGyroReset2);
        JoystickButton operatorGyroReset = new JoystickButton(m_xBoxOperator, ButtonConstants.OperatorGyroReset);
        shuffleboardGyroResetO.setString("O-Button " + Constants.ButtonConstants.OperatorGyroReset);
        JoystickButton operatorGyroReset2 = new JoystickButton(m_xBoxOperator, ButtonConstants.OperatorGyroReset2);

        POVButton driverTurretLeftButton = new POVButton(m_xBoxDriver, ButtonConstants.TurretLeftPOV);
        POVButton driverTurretRightButton = new POVButton(m_xBoxDriver, ButtonConstants.TurretRightPOV);

        SpectrumAxisButton operatorTurretLeft = new SpectrumAxisButton(m_xBoxOperator,
                ButtonConstants.OperatorTurretAxis,
                ButtonConstants.JoystickLeftThreshold, SpectrumAxisButton.ThresholdType.GREATER_THAN);
        SpectrumAxisButton operatorTurretRight = new SpectrumAxisButton(m_xBoxOperator,
                ButtonConstants.OperatorTurretAxis,
                ButtonConstants.JoystickRightThreshold, SpectrumAxisButton.ThresholdType.LESS_THAN);
        SpectrumAxisButton operatorTurretAutoFind = new SpectrumAxisButton(m_xBoxOperator,
                ButtonConstants.OperatorTurretFindAxis, ButtonConstants.JoystickUpThreshold,
                SpectrumAxisButton.ThresholdType.GREATER_THAN);
        SpectrumAxisButton operatorTurretAutoFindStop = new SpectrumAxisButton(m_xBoxOperator,
                ButtonConstants.OperatorTurretFindAxis, ButtonConstants.JoystickDownThreshold,
                SpectrumAxisButton.ThresholdType.LESS_THAN);

        SpectrumAxisButton operatorClimbUp = new SpectrumAxisButton(m_xBoxOperator, ButtonConstants.OperatorClimbAxis,
                ButtonConstants.JoystickUpThreshold, SpectrumAxisButton.ThresholdType.LESS_THAN);
        SpectrumAxisButton operatorClimbDown = new SpectrumAxisButton(m_xBoxOperator, ButtonConstants.OperatorClimbAxis,
                ButtonConstants.JoystickDownThreshold, SpectrumAxisButton.ThresholdType.GREATER_THAN);

        JoystickButton operatorPivotArm = new JoystickButton(m_xBoxOperator, ButtonConstants.OperatorPivotArm);
        shuffleboardOperatorPivotArm.setString("O-Button " + Constants.ButtonConstants.OperatorPivotArm);

        JoystickButton operatorManualShoot = new JoystickButton(m_xBoxOperator, ButtonConstants.OperatorShootManual);
        shuffleboardOperatorManualShoot.setString("O-Button " + Constants.ButtonConstants.OperatorShootManual);

        JoystickButton operatorClimbHooks = new JoystickButton(m_xBoxOperator, ButtonConstants.OperatorClimbHook);

        POVButton operatorShoot1Button = new POVButton(m_xBoxOperator, ButtonConstants.OperatorShooter1POV);
        POVButton operatorShoot2Button = new POVButton(m_xBoxOperator, ButtonConstants.OperatorShooter2POV);
        POVButton operatorShoot3Button = new POVButton(m_xBoxOperator, ButtonConstants.OperatorShooter3POV);
        POVButton operatorShoot4Button = new POVButton(m_xBoxOperator, ButtonConstants.OperatorShooter4POV);

        // turretAutoFind.whenHeld(turretAutoCommand);

        // aimAndSpin.whenHeld(aimAndSpinCommand);
        // aimAndSpin2.whenHeld(aimAndSpinCommand);

        driverAutoShoot.whenHeld(LoadandShootCommand);
        operatorAutoShoot.whenHeld(LoadandShootCommand);

        driverIntakeIn.whenHeld(intakeInCommand);
        operatorIntakeIn.whenHeld(intakeInCommand);

        driverIntakeOut.whenHeld(intakeOutCommand);
        operatorIntakeOut.whenHeld(intakeOutCommand);

        driverIntakeArm.whenHeld(intakeArmCommand);
        operatorIntakeArm.whenHeld(intakeArmCommand);

        driverGyroReset.whenPressed(gyroResetCommand);
        driverGyroReset2.whenPressed(gyroResetCommand);
        operatorGyroReset.whenPressed(gyroResetCommand);
        operatorGyroReset2.whenPressed(gyroResetCommand);

        driverTurretLeftButton.whenHeld(turretLeftCommand);
        driverTurretRightButton.whenHeld(turretRightCommand);

        operatorTurretLeft.whenHeld(turretLeftCommand);
        operatorTurretRight.whenHeld(turretRightCommand);
        operatorTurretAutoFind.whenHeld(turretAutoCommand);
        operatorTurretAutoFindStop.whenHeld(turretStopCommand); // should force auto command to stop since same
                                                                // subsystem
                                                                // requirements
        operatorClimbUp.whenHeld(climbUpCommand);
        operatorClimbDown.whenHeld(climbDownCommand);

        operatorPivotArm.whenHeld(climbArmsCommand);

        // operatorClimbHooks.whenPressed(releaseAndResetClimb);
        operatorClimbHooks.whenHeld(climbHooksCommand);
        operatorClimbHooks.whenReleased(releaseAndResetClimb);
        operatorManualShoot.whenHeld(ManualShootCommand);

        operatorShoot1Button.whenHeld(ManualShooter1Command);
        operatorShoot2Button.whenHeld(ManualShooter2Command);
        operatorShoot3Button.whenHeld(ManualShooter3Command);
        operatorShoot4Button.whenHeld(ManualShooter4Command);
        // latchButton.whenPressed(latchCommand);
    }

    public Command getAutonomousCommand() {
        Command ReturnCommand = autonomousChooser.getCommand(this);
        System.out.println(ReturnCommand);
        return ReturnCommand;
    }

    private Double getDriveForwardAxis() {
        return m_xBoxDriver.getLeftY();
    }

    private Double getDriveStrafeAxis() {
        return m_xBoxDriver.getLeftX();
    }

    private Double getDriveRotationAxis() {
        return m_xBoxDriver.getRightX();
    }

    public Limelight getVisionSubsystem() {
        return m_Limelight;
    }

    public void disableLimelights() {
        m_Limelight.turnLEDOff();
    }

    public void enableLimelights() {
        m_Limelight.turnLEDOn();
    }

    public TurretSubsystem getTurretSubsystem() {
        return m_turret;
    }

    public void refreshSmartDashboard() {

        shuffleboardPDPStickyCANFaults.setBoolean(m_robotPDH.getStickyFaults().CanWarning);
        shuffleboardPDPTotalCurrent.setDouble(m_robotPDH.getTotalCurrent());
        shuffleboardPDPCurrentC0.setDouble(m_robotPDH.getCurrent(0));
        shuffleboardPDPCurrentC1.setDouble(m_robotPDH.getCurrent(1));
        shuffleboardPDPCurrentC2.setDouble(m_robotPDH.getCurrent(2));
        shuffleboardPDPCurrentC3.setDouble(m_robotPDH.getCurrent(3));
        shuffleboardPDPCurrentC4.setDouble(m_robotPDH.getCurrent(4));
        shuffleboardPDPCurrentC5.setDouble(m_robotPDH.getCurrent(5));
        shuffleboardPDPCurrentC6.setDouble(m_robotPDH.getCurrent(6));
        shuffleboardPDPCurrentC7.setDouble(m_robotPDH.getCurrent(7));
        shuffleboardPDPCurrentC8.setDouble(m_robotPDH.getCurrent(8));
        shuffleboardPDPCurrentC9.setDouble(m_robotPDH.getCurrent(9));
        shuffleboardPDPCurrentC10.setDouble(m_robotPDH.getCurrent(10));
        shuffleboardPDPCurrentC11.setDouble(m_robotPDH.getCurrent(11));
        shuffleboardPDPCurrentC12.setDouble(m_robotPDH.getCurrent(12));
        shuffleboardPDPCurrentC13.setDouble(m_robotPDH.getCurrent(13));
        shuffleboardPDPCurrentC14.setDouble(m_robotPDH.getCurrent(14));
        shuffleboardPDPCurrentC15.setDouble(m_robotPDH.getCurrent(15));
        shuffleboardPDPCurrentC16.setDouble(m_robotPDH.getCurrent(16));
        shuffleboardPDPCurrentC17.setDouble(m_robotPDH.getCurrent(17));
        shuffleboardPDPCurrentC18.setDouble(m_robotPDH.getCurrent(18));
        shuffleboardPDPCurrentC19.setDouble(m_robotPDH.getCurrent(19));
        shuffleboardPDPCurrentC20.setDouble(m_robotPDH.getCurrent(20));
        shuffleboardPDPCurrentC21.setDouble(m_robotPDH.getCurrent(21));
        shuffleboardPDPCurrentC22.setDouble(m_robotPDH.getCurrent(22));
        shuffleboardPDPCurrentC23.setDouble(m_robotPDH.getCurrent(23));
        m_Limelight.update();
        if (m_Limelight.isTargetAvailible()) {
            networkTableEntryVisionDistance.setDouble(m_Limelight.getDistance());
        }
        else {
            networkTableEntryVisionDistance.setDouble(0);
        }
        shuffleboardShooterTopVel.setDouble(m_Shooter.getTopMotorVelocity());
        shuffleboardShooterBottomVel.setDouble(m_Shooter.getBottomMotorVelocity());
        shuffleboardTurretPos.setString(" " + m_turret.getPosition());
        shuffleboardLeftLimit.setBoolean(m_turret.hitLeftLimit());
        shuffleboardRightLimit.setBoolean(m_turret.hitRightLimit());
        m_Shooter.setMultipler(shuffleobardShooterMultipler.getDouble(1.0));
    }

    public void disabledPerioidicUpdates() {
        shuffleboardLeftLimit.setBoolean(m_turret.hitLeftLimit());
        shuffleboardRightLimit.setBoolean(m_turret.hitRightLimit());
    }

    public void resetTurret() {
        if (m_turretHasReset == false) {
            zTurretResetCommand resetTurret = new zTurretResetCommand(getTurretSubsystem(), Constants.turretInitSpeed,
                    Constants.turretHomeSpeed, Constants.turretHomePos);
            CommandScheduler.getInstance().schedule(resetTurret);
            m_turretHasReset = true;
        }
    }

    public DrivetrainSubsystem getDrivetrainSubsystem() {
        return drivetrainSubsystem;
    }

    public ShooterSubsystem getShooterSubsystem() {
        return m_Shooter;
    }

    public IntakeSubsystem[] getIntakeSubsystems() {
        return new IntakeSubsystem[] { m_Intake, m_IntakeInner };
    }

    public PneumaticsSubsystem getPneumaticsSubsystem() {
        return m_Pnuematics;
    }

    public ConveyorSubsystem getConveyorSubsystem() {
        return m_Conveyor;
    }

    public FeederSubsystem getFeederSubsystem() {
        return m_Feeder;
    }
}
