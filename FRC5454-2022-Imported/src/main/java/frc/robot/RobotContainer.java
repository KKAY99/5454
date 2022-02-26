// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

 
import java.util.Map;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.classes.SpectrumAxisButton;
import frc.robot.commands.*;
import frc.robot.common.drivers.NavX;
import frc.robot.subsystems.*;
import frc.robot.Constants.AutoModes;
import frc.robot.Constants.ButtonConstants;
import frc.robot.Constants.InputControllers;
import frc.robot.Constants.zAutomation;
import frc.robot.classes.Limelight;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.networktables.NetworkTableEntry;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.smartdashboard.*;
import edu.wpi.first.wpilibj.PowerDistribution;
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
  //  private AHRS m_ahrs = new AHRS(SPI.Port.kMXP);
    private NavX m_NavX = new NavX(SPI.Port.kMXP);
   // private final DriveSubsystem m_RobotDrive = new DriveSubsystem(m_ahrs);
   private final DrivetrainSubsystem m_RobotDrive = new DrivetrainSubsystem(m_NavX); 
   //private final SwerveSubsystem m_RobotDrive = new SwerveSubsystem();
    private final Limelight m_Limelight = new Limelight(Constants.LimeLightValues.targetHeight, Constants.LimeLightValues.limelightHeight, Constants.LimeLightValues.limelightAngle);
  

    // Shooter(Integer BottomMotorPort, Integer TopMotorPort)
    private final ShooterSubsystem m_Shooter = new ShooterSubsystem(Constants.TopShooterPort,Constants.BottomShooterPort);
    private final ConveyorSubsystem m_Conveyor = new ConveyorSubsystem(Constants.ConveyorPort);
    private final FeederSubsystem m_Feeder= new FeederSubsystem(Constants.FeederPort);
    private final IntakeSubsystem m_Intake = new IntakeSubsystem(Constants.IntakePort);
    private final ClimbSubsystem m_Climb = new ClimbSubsystem(Constants.ClimberPort,Constants.LimitSwitches.ClimberBottom);
    private final PneumaticsSubsystem m_Pnuematics = new PneumaticsSubsystem(Constants.Pneumatics.CompressorID);
    private final TurretSubsystem m_turret = new TurretSubsystem(Constants.TurretPort,Constants.LimitSwitches.TurretLeft,Constants.LimitSwitches.TurretRight);
    // #region Shuffleboard

    private final zSpinLoadShootCommand autoLoadShoot = new zSpinLoadShootCommand(m_Shooter, m_Conveyor, m_Feeder,AutoModes.AutoShotTopSpeed, AutoModes.AutoShotBottomSpeed,AutoModes.AutoMinVelocity); 
       

    private final PowerDistribution m_robotPDH = new PowerDistribution(1, PowerDistribution.ModuleType.kRev);

    // #region Create Shuffleboard Tabs
    private static ShuffleboardTab SwerveTab = Shuffleboard.getTab("Swerve");
    private static ShuffleboardTab SwerveEncoders = Shuffleboard.getTab("SwerveEncoders");
    private static ShuffleboardTab AutoTab = Shuffleboard.getTab("Auto");
    private static ShuffleboardTab ShooterTab=Shuffleboard.getTab("Shooter");
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

    static NetworkTableEntry shuffleboardDrive=ControlTab.add("Drive Control","Left Stick").getEntry();
    static NetworkTableEntry shuffleboarRotate=ControlTab.add("Swerve Control","Right Stick").getEntry();
    static NetworkTableEntry shuffleboardIntakeIn=ControlTab.add("Intake In","").getEntry();
    static NetworkTableEntry shuffleboardIntakeOut=ControlTab.add("Intake Out","").getEntry();
    static NetworkTableEntry shuffleboardConveyorIn=ControlTab.add("Conveyor In","").getEntry();
    static NetworkTableEntry shuffleboardConveyorOut=ControlTab.add("Conveyor Out","").getEntry();
    static NetworkTableEntry shuffleboardBallFeedUp=ControlTab.add("Feed Up","").getEntry();
    static NetworkTableEntry shuffleboardBallFeedDown=ControlTab.add("Feed Down","").getEntry();
    static NetworkTableEntry shuffleboardIntakeIn2=ControlTab.add("Intake In-R","").getEntry();
    static NetworkTableEntry shuffleboardIntakeOut2=ControlTab.add("Intake Out-R","").getEntry();
    static NetworkTableEntry shuffleboardConveyorIn2=ControlTab.add("Conveyor In-R","").getEntry();
    static NetworkTableEntry shuffleboardConveyorOut2=ControlTab.add("Conveyor Out-R","").getEntry();
    static NetworkTableEntry shuffleboardBallFeedUp2=ControlTab.add("Ball Feed Up-R","").getEntry();
    static NetworkTableEntry shuffleboardBallFeedDown2=ControlTab.add("Ball Feed Down-R","").getEntry();
    static NetworkTableEntry shuffleboardManualShoot=ControlTab.add("Manual Shoot","").getEntry(); 
    static NetworkTableEntry shuffleboardManualShoot2=ControlTab.add("Manual Shoot-R","").getEntry();
    static NetworkTableEntry shuffleboardAutoShoot=ControlTab.add("Auto Shoot","").getEntry();
    static NetworkTableEntry shuffleboardAimAndShoot=ControlTab.add("Aim and Shoot","").getEntry();
    static NetworkTableEntry shuffleboardAimAndShoot2=ControlTab.add("Aim and Shoot-R","").getEntry();
    static NetworkTableEntry shuffleboardTurretTurn=ControlTab.add("Turret Turn","").getEntry();
    static NetworkTableEntry shuffleboardClimbLift=ControlTab.add("Climb Lift","").getEntry();
    static NetworkTableEntry shuffleboardClimbLower=ControlTab.add("Climb Down","").getEntry();
    static NetworkTableEntry shuffleboardIntakeArm=ControlTab.add("Intake Arm","").getEntry();
    static NetworkTableEntry shuffleboardLatch=ControlTab.add("Latch","").getEntry();
    static NetworkTableEntry shuffleboardPDPStickyCANFaults=PDPTab.add("Sticky CAN Faults","").getEntry();
    static NetworkTableEntry shuffleboardPDPTotalCurrent=PDPTab.add("Total Current","").getEntry();
    static NetworkTableEntry shuffleboardPDPCurrentC0=PDPTab.add("Current C0","").getEntry();
    static NetworkTableEntry shuffleboardPDPCurrentC1=PDPTab.add("Current C1","").getEntry();
    static NetworkTableEntry shuffleboardPDPCurrentC2=PDPTab.add("Current C2","").getEntry();
    static NetworkTableEntry shuffleboardPDPCurrentC3=PDPTab.add("Current C3","").getEntry();
    static NetworkTableEntry shuffleboardPDPCurrentC4=PDPTab.add("Current C4","").getEntry();
    static NetworkTableEntry shuffleboardPDPCurrentC5=PDPTab.add("Current C5","").getEntry();
    static NetworkTableEntry shuffleboardPDPCurrentC6=PDPTab.add("Current C6","").getEntry();
    static NetworkTableEntry shuffleboardPDPCurrentC7=PDPTab.add("Current C7","").getEntry();
    static NetworkTableEntry shuffleboardPDPCurrentC8=PDPTab.add("Current C8","").getEntry();
    static NetworkTableEntry shuffleboardPDPCurrentC9=PDPTab.add("Current C9","").getEntry();
    static NetworkTableEntry shuffleboardPDPCurrentC10=PDPTab.add("Current C10","").getEntry();  
    static NetworkTableEntry shuffleboardPDPCurrentC11=PDPTab.add("Current C11","").getEntry();
    static NetworkTableEntry shuffleboardPDPCurrentC12=PDPTab.add("Current C12","").getEntry();
    static NetworkTableEntry shuffleboardPDPCurrentC13=PDPTab.add("Current C13","").getEntry();
    static NetworkTableEntry shuffleboardPDPCurrentC14=PDPTab.add("Current C14","").getEntry();
    static NetworkTableEntry shuffleboardPDPCurrentC15=PDPTab.add("Current C15","").getEntry();
    static NetworkTableEntry shuffleboardPDPCurrentC16=PDPTab.add("Current C16","").getEntry();
    static NetworkTableEntry shuffleboardPDPCurrentC17=PDPTab.add("Current C17","").getEntry();
    static NetworkTableEntry shuffleboardPDPCurrentC18=PDPTab.add("Current C18","").getEntry();
    static NetworkTableEntry shuffleboardPDPCurrentC19=PDPTab.add("Current C19","").getEntry();
    static NetworkTableEntry shuffleboardPDPCurrentC20=PDPTab.add("Current C20","").getEntry();
    static NetworkTableEntry shuffleboardPDPCurrentC21=PDPTab.add("Current C21","").getEntry();
    static NetworkTableEntry shuffleboardPDPCurrentC22=PDPTab.add("Current C22","").getEntry();
    static NetworkTableEntry shuffleboardPDPCurrentC23=PDPTab.add("Current C23","").getEntry();
    
    static NetworkTableEntry shuffleboardTurretPos=ShooterTab.add("Turret Position","").getEntry();
    
    static NetworkTableEntry shuffleboardLeftLimit=ShooterTab.add("Left Limit","").getEntry();
    static NetworkTableEntry shuffleboardRightLimit=ShooterTab.add("Right Limit","").getEntry();
    static NetworkTableEntry shuffleboardShooterTop=ShooterTab.add("Top Speed",Constants.AutoModes.AutoShotTopSpeed).getEntry();
    static NetworkTableEntry shuffleboardShooterBottom=ShooterTab.add("Bottom Speed",Constants.AutoModes.AutoShotBottomSpeed).getEntry();

    

    static String ShuffleboardLogString;
    // #endregion
    // #endregion

    // private final ExampleCommand m_autoCommand = new
    // ExampleCommand(m_exampleSubsystem);

    private XboxController m_xBoxDriver = new XboxController(InputControllers.kXboxDrive);
    private XboxController m_xBoxOperator = new XboxController(InputControllers.kXboxOperator);
  
    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // Configure the button bindings
        configureButtonBindings();

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
        final ConveyorCommand conveyorDownCommand = new ConveyorCommand(m_Conveyor,Constants.conveyorDownSpeed);
        //final IntakeCommand intakeInCommand = new IntakeCommand(m_Intake,Constants.intakeSpeed);
        //final IntakeCommand intakeOutCommand = new IntakeCommand(m_Intake,-Constants.intakeSpeed);
        final zIntakeConveyCommand intakeInCommand = new zIntakeConveyCommand(m_Intake,Constants.intakeSpeed,m_Conveyor,Constants.conveyorUpSpeed,m_Feeder,-Constants.FeederSpeed);
        final zIntakeConveyCommand intakeOutCommand = new zIntakeConveyCommand(m_Intake,-Constants.intakeSpeed,m_Conveyor,Constants.conveyorDownSpeed,m_Feeder,Constants.FeederSpeed);
        final ShooterCommand shootCommand = new ShooterCommand(m_Shooter,m_Limelight,AutoModes.AutoShotTopSpeed,AutoModes.AutoShotBottomSpeed,false);
        final FeederCommand feedUpCommand=new FeederCommand(m_Feeder,Constants.FeederSpeed);
        final ClimbCommand climbUpCommand=new ClimbCommand(m_Climb,Constants.climbUpSpeed);
        final ClimbCommand climbDownCommand=new ClimbCommand(m_Climb,Constants.climbDownSpeed);
       
        final IntakeArmCommand intakeArmCommand = new IntakeArmCommand(m_Pnuematics);
        final TurretCommand turretLeftCommand = new TurretCommand(m_turret,Constants.turretSpeed);
        final TurretCommand turretRightCommand = new TurretCommand(m_turret,-Constants.turretSpeed);
        //final LatchCommand latchCommand =new LatchCommand(m_Pnuematics);
         
        JoystickButton aimAndSpin = new JoystickButton(m_xBoxDriver, ButtonConstants.AimandShoot);
        shuffleboardAimAndShoot.setString("Left-Button " + ButtonConstants.AimandShoot);
        
        JoystickButton aimAndSpin2 = new JoystickButton(m_xBoxOperator, ButtonConstants.AimandShoot);
        shuffleboardAimAndShoot2.setString("Right-Button " + ButtonConstants.AimandShoot);
        
        JoystickButton manualShoot = new JoystickButton(m_xBoxDriver, ButtonConstants.ManualShoot);        
        shuffleboardManualShoot.setString("Left-Button " + ButtonConstants.ManualShoot);
      
        JoystickButton manualShoot2 = new JoystickButton(m_xBoxOperator, ButtonConstants.ManualShoot);        
        shuffleboardManualShoot2.setString("Right-Button " + ButtonConstants.ManualShoot);
      
        // Creates a new JoystickButton object for button 6 (xBox rightBumper) on
        // m_RobotDrive
   
        JoystickButton intakeInButton= new JoystickButton(m_xBoxDriver, ButtonConstants.IntakeIn);
        shuffleboardIntakeIn.setString("Left-Button " + ButtonConstants.IntakeIn);

        JoystickButton intakeIn2Button= new JoystickButton(m_xBoxOperator, ButtonConstants.IntakeIn);
        shuffleboardIntakeIn2.setString("Right-Button " + ButtonConstants.IntakeIn);

        JoystickButton intakeOutButton = new JoystickButton(m_xBoxDriver, ButtonConstants.IntakeOut);
        shuffleboardIntakeOut.setString("Left-Button " + ButtonConstants.IntakeOut);

        JoystickButton intakeOut2Button = new JoystickButton(m_xBoxOperator, ButtonConstants.IntakeOut);
        shuffleboardIntakeOut2.setString("Right-Button " + ButtonConstants.IntakeOut);

        JoystickButton feederUpButton = new JoystickButton(m_xBoxDriver, ButtonConstants.FeederUp);
        shuffleboardBallFeedUp.setString("Left-Button " + ButtonConstants.FeederUp);
        
        JoystickButton feederUp2Button = new JoystickButton(m_xBoxOperator, ButtonConstants.FeederUp);
        shuffleboardBallFeedUp.setString("Right-Button " + ButtonConstants.FeederUp);


        JoystickButton climbUpButton = new JoystickButton(m_xBoxDriver,ButtonConstants.ClimberUp);
        shuffleboardClimbLift.setString("Left-Button " + ButtonConstants.ClimberUp);
        
        JoystickButton intakeArmButton = new JoystickButton(m_xBoxDriver,ButtonConstants.IntakeArm);
        shuffleboardIntakeArm.setString("Left-Button " + ButtonConstants.IntakeArm);
        

        JoystickButton latchButton = new JoystickButton(m_xBoxDriver,ButtonConstants.Latch);
        shuffleboardLatch.setString("Left-Button " + ButtonConstants.Latch);

        SpectrumAxisButton conveyorUpButton = new SpectrumAxisButton(m_xBoxDriver,ButtonConstants.ConveyerUpAxis,ButtonConstants.TriggerThreshold,SpectrumAxisButton.ThresholdType.GREATER_THAN);
        shuffleboardConveyorIn.setString("Left-Trigger " + ButtonConstants.ConveyerUpAxis);
     
        SpectrumAxisButton conveyorUp2Button = new SpectrumAxisButton(m_xBoxOperator,ButtonConstants.ConveyerUpAxis,ButtonConstants.TriggerThreshold,SpectrumAxisButton.ThresholdType.GREATER_THAN);
        shuffleboardConveyorIn.setString("Right-Trigger " + ButtonConstants.ConveyerUpAxis);
        
        SpectrumAxisButton conveyorDownButton = new SpectrumAxisButton(m_xBoxDriver,ButtonConstants.ConveyerDownAxis,ButtonConstants.TriggerThreshold,SpectrumAxisButton.ThresholdType.GREATER_THAN);
        shuffleboardConveyorOut.setString("Left-Trigger " + ButtonConstants.ConveyerDownAxis);
     
        SpectrumAxisButton conveyorDown2Button = new SpectrumAxisButton(m_xBoxOperator,ButtonConstants.ConveyerDownAxis,ButtonConstants.TriggerThreshold,SpectrumAxisButton.ThresholdType.GREATER_THAN);
        shuffleboardConveyorOut.setString("Right-Trigger " + ButtonConstants.ConveyerDownAxis);
     

        POVButton turretLeftButton=new POVButton(m_xBoxDriver,ButtonConstants.TurretLeftPOV);
        POVButton turretRightButton=new POVButton(m_xBoxDriver,ButtonConstants.TurretRightPOV); 
  
        POVButton climbUpPOV=new POVButton(m_xBoxOperator,ButtonConstants.ClimbUpPOV);
        POVButton climbDownPOV=new POVButton(m_xBoxOperator,ButtonConstants.ClimbDownPOV); 
  
        // new POVButton(m_leftJoystick,0).whenHeld(new RobotMoveCommand(m_RobotDrive,-Constants.kSlowMoveLeft,Constants.kSlowMoveRight));    
     
        aimAndSpin.whenHeld(aimAndSpinCommand);
        aimAndSpin2.whenHeld(aimAndSpinCommand);
        
        //manualShoot.whenHeld(shootCommand);
        manualShoot.whenHeld(autoLoadShoot);
        manualShoot2.whenHeld(autoLoadShoot);
        
        conveyorUpButton.whenHeld(conveyorUpCommand);
        conveyorDownButton.whenHeld(conveyorDownCommand);
        conveyorUp2Button.whenHeld(conveyorUpCommand);
        conveyorDown2Button.whenHeld(conveyorDownCommand);
     
        intakeOutButton.whenHeld(intakeOutCommand);
        intakeInButton.whenHeld(intakeInCommand); 
        intakeOut2Button.whenHeld(intakeOutCommand);
        intakeIn2Button.whenHeld(intakeInCommand); 
        feederUpButton.whenHeld(feedUpCommand);
        feederUp2Button.whenHeld(feedUpCommand);
        climbUpButton.whenHeld(climbUpCommand);
        turretLeftButton.whenHeld(turretLeftCommand);
        turretRightButton.whenHeld(turretRightCommand);
        climbUpPOV.whenHeld(climbUpCommand);
        climbDownPOV.whenHeld(climbDownCommand);
        
        intakeArmButton.whenHeld(intakeArmCommand);    
    
        //latchButton.whenPressed(latchCommand);
   }

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
             autoCommand= new AutoMoveCommand(m_RobotDrive,0,AutoModes.LeaveTarmacDistance);
            break;
          case AutoModes.autoMoveShoot:
            autoCommand=new SequentialCommandGroup(
                    new zSpinLoadShootCommand(m_Shooter, m_Conveyor, m_Feeder,AutoModes.AutoShotTopSpeed, AutoModes.AutoShotBottomSpeed,AutoModes.AutoMinVelocity),
                    new AutoMoveCommand(m_RobotDrive,0,AutoModes.LeaveTarmacDistance));
            break;
          case AutoModes.autoMoveBackwardsOutake:
          autoCommand=new SequentialCommandGroup(
                new zIntakeTimeCommand(m_Intake, -Constants.intakeSpeed,0,true),
                new AutoMoveCommand(m_RobotDrive,0,AutoModes.LeaveTarmacDistance)           
                );
            break;
          case AutoModes.autoMoveBackwardsShot:
          
                  autoCommand=new SequentialCommandGroup(
                new zSpinLoadShootCommand(m_Shooter, m_Conveyor, m_Feeder,AutoModes.AutoShotTopSpeed, AutoModes.AutoShotBottomSpeed,AutoModes.AutoMinVelocity),
                new AutoMoveCommand(m_RobotDrive,180,AutoModes.LeaveTarmacDistance));

            break;
          case AutoModes.autoMoveShootMoveGrab:
                autoCommand=new SequentialCommandGroup(
                new zSpinLoadShootCommand(m_Shooter, m_Conveyor, m_Feeder,AutoModes.AutoShotTopSpeed, AutoModes.AutoShotBottomSpeed,AutoModes.AutoMinVelocity),
                new zIntakeTimeCommand(m_Intake, Constants.intakeSpeed,0,true),
                new AutoMoveCommand(m_RobotDrive,0,AutoModes.LeaveTarmacDistance),
                new zIntakeTimeCommand(m_Intake, Constants.intakeSpeed,zAutomation.intakeTime,false)
                );
            break;
          case AutoModes.autoMoveShootMoveGrabShot1:
            System.out.println("Auto Shot Move Grab Shoot Pay Executing");
            autoCommand=new SequentialCommandGroup(
                new zSpinLoadShootCommand(m_Shooter, m_Conveyor,m_Feeder, AutoModes.AutoShotTopSpeed, AutoModes.AutoShotBottomSpeed,AutoModes.AutoMinVelocity),
                new zIntakeTimeCommand(m_Intake, Constants.intakeSpeed,0,true),
                new AutoMoveCommand(m_RobotDrive,0,AutoModes.LeaveTarmacDistance),
                new zIntakeTimeCommand(m_Intake, Constants.intakeSpeed,zAutomation.intakeTime),
                new zSpinLoadShootCommand(m_Shooter, m_Conveyor, m_Feeder,AutoModes.AutoShotTopSpeed, AutoModes.AutoShotBottomSpeed,AutoModes.AutoMinVelocity));
            break;
          case AutoModes.autoMoveShotMoveGrabMoveLeftGrabShot2:
            autoCommand=new SequentialCommandGroup(
                new zSpinLoadShootCommand(m_Shooter, m_Conveyor, m_Feeder,AutoModes.AutoShotTopSpeed, AutoModes.AutoShotBottomSpeed,AutoModes.AutoMinVelocity),
                new AutoMoveCommand(m_RobotDrive,0,AutoModes.LeaveTarmacDistance),
                new zIntakeTimeCommand(m_Intake, Constants.intakeSpeed,zAutomation.intakeTime),
                new zSpinLoadShootCommand(m_Shooter, m_Conveyor, m_Feeder,AutoModes.AutoShotTopSpeed, AutoModes.AutoShotBottomSpeed,AutoModes.AutoMinVelocity),
                new AutoMoveCommand(m_RobotDrive,270,AutoModes.ball2Distance),
                new zSpinLoadShootCommand(m_Shooter, m_Conveyor,m_Feeder, AutoModes.AutoShotTopSpeed, AutoModes.AutoShotBottomSpeed,AutoModes.AutoMinVelocity));            break;
          case AutoModes.autoMoveShotMoveGrabMoveRightGrabShot2:
          autoCommand=new SequentialCommandGroup(
                new zSpinLoadShootCommand(m_Shooter, m_Conveyor, m_Feeder,AutoModes.AutoShotTopSpeed, AutoModes.AutoShotBottomSpeed,AutoModes.AutoMinVelocity),
                new AutoMoveCommand(m_RobotDrive,0,AutoModes.LeaveTarmacDistance),
                new zIntakeTimeCommand(m_Intake, Constants.intakeSpeed,zAutomation.intakeTime),
                new zSpinLoadShootCommand(m_Shooter, m_Conveyor, m_Feeder,AutoModes.AutoShotTopSpeed, AutoModes.AutoShotBottomSpeed,AutoModes.AutoMinVelocity),
                new AutoMoveCommand(m_RobotDrive,90,AutoModes.ball2Distance),
                new zSpinLoadShootCommand(m_Shooter, m_Conveyor, m_Feeder,AutoModes.AutoShotTopSpeed, AutoModes.AutoShotBottomSpeed,AutoModes.AutoMinVelocity));

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
    public void refreshSmartDashboard()
    {  
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
        frontLeftAngle.setDouble(m_RobotDrive.getFrontLeftAngle());
        frontRightAngle.setDouble(m_RobotDrive.getFrontRightAngle());
        backLeftAngle.setDouble(m_RobotDrive.getBackLeftAngle());
        backRightAngle.setDouble(m_RobotDrive.getbackRightAngle());
        if (m_turret.hitLeftLimit()){
                shuffleboardLeftLimit.setString("True");                                
        }else
        {
                shuffleboardLeftLimit.setString("False");

        }
        if (m_turret.hitRightLimit()){
                shuffleboardRightLimit.setString("True");                                
        }else
        {
                shuffleboardRightLimit.setString("False");
        }
        m_Limelight.update();
        if(m_Limelight.isTargetAvailible()){              
                networkTableEntryVisionDistance.setDouble(m_Limelight.getDistance());
        }else {
                networkTableEntryVisionDistance.setDouble(0);
        }
        shuffleboardTurretPos.setString(" " + m_turret.getPosition());
        shuffleboardLeftLimit.setBoolean(m_turret.hitLeftLimit());
        shuffleboardRightLimit.setBoolean(m_turret.hitRightLimit());
        // allow user to update shooter spped
        double topSpeed=shuffleboardShooterTop.getDouble(0);
        double bottomSpeed=shuffleboardShooterBottom.getDouble(0);
        if((topSpeed!=0) || (bottomSpeed!=0)){
                autoLoadShoot.changeSpeeds(topSpeed, bottomSpeed, topSpeed);
        }

        
    }
    public void disableLimelights(){
            m_Limelight.turnLEDOff();
    }
    public void enableLimelights(){
            m_Limelight.turnLEDOn();
    }
    public void resetDriveModes(){
//        m_RobotDrive.resetDriveMode();
    }
}
