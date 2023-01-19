// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

 
import java.util.Map;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.classes.SpectrumAxisButton;
import frc.robot.commands.*;
import frc.robot.common.drivers.NavX;
import frc.robot.subsystems.*;
import frc.robot.Constants.AutoModes;
import frc.robot.Constants.ButtonConstants;
import frc.robot.Constants.InputControllers;
import frc.robot.Constants.SwerveDriveGBConfig;
import frc.robot.Constants.zAutomation;
import frc.robot.Constants.LEDS.Colors;
import frc.robot.classes.Limelight;
import frc.robot.classes.LEDStrip;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.networktables.GenericEntry;
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
   //private final DrivetrainSubsystem m_RobotDrive = new DrivetrainSubsystem(m_NavX); 
   private final SwerveDriveGB m_RobotDrive= new SwerveDriveGB(m_NavX);
    private final Limelight m_Limelight = new Limelight(Constants.LimeLightValues.targetHeight, Constants.LimeLightValues.limelightHeight, Constants.LimeLightValues.limelightAngle,Constants.LimeLightValues.kVisionXOffset);
    
   //  private final LEDStrip m_ledStrip = new LEDStrip(Constants.LEDS.PORT, Constants.LEDS.COUNT);
     //KK Clean up Disbled
     private static enum LEDMode
     {
                     NOTSET,DISBLED, AUTOMODE, OFFTARGET, OFFTARGETSWEET, ONTARGETSWEET,ONTARGET,SHOOTING,CLIMBING,TELEOP;	
     }
     private LEDMode m_LEDMode=LEDMode.DISBLED;
     private boolean m_ledFlash=false;
     private boolean m_ledFlashMode=false;
     private int m_ledFlashDelayCount=0;
     //KK Clean Up
     private static final int LEDMODE_WAVE = 0;
     private static final int LEDMODE_BAR = 1;
     private static final int LEDMODE_RAINBOW = 2;
     private static final int LEDMODE_SOLID = 3;
     private static final int LEDMODE_OFF = 4;
     private LEDMode m_oldLEDmode=LEDMode.NOTSET;  
     
    // Shooter(Integer BottomMotorPort, Integer TopMotorPort)
    //private final ShooterSubsystemVoltage m_Shooter = new ShooterSubsystemVoltage(Constants.TopShooterPort,Constants.BottomShooterPort,Constants.shooterPrimedSpeed);
    private final IntakeSubsystem m_Intake = new IntakeSubsystem(Constants.IntakePort);
   
    private final PneumaticsSubsystem m_Pnuematics = new PneumaticsSubsystem(Constants.Pneumatics.CompressorID,Constants.Pneumatics.IntakeArmPort,Constants.Pneumatics.ClimbPort);
    
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

   
    static GenericEntry GenericEntryVisionDistance = ShooterTab.add("Vision Distance", 0)
            .withWidget(BuiltInWidgets.kNumberBar).withSize(2, 2).getEntry();

    
    static GenericEntry GenericEntryFrontLeftSpeed = SwerveTab.add("FL Speed", 0)
            .withWidget(BuiltInWidgets.kVoltageView)
            .withProperties(Map.of("Min", 0, "Max", 1, "Center", 0, "Orientation", "VERTICAL"))
            .withPosition(1, 0).withSize(2, 5).getEntry();

    static GenericEntry GenericEntryFrontRightSpeed = SwerveTab.add("FR Speed", 0)
            .withWidget(BuiltInWidgets.kVoltageView)
            .withProperties(Map.of("Min", 0, "Max", 1, "Center", 0, "Orientation", "VERTICAL"))
            .withPosition(14, 0).withSize(2, 5).getEntry();

    static GenericEntry GenericEntryBackLeftSpeed = SwerveTab.add("BL Speed", 0)
            .withWidget(BuiltInWidgets.kVoltageView)
            .withProperties(Map.of("Min", 0, "Max", 1, "Center", 0, "Orientation", "VERTICAL"))
            .withPosition(1, 5).withSize(2, 5).getEntry();

    static GenericEntry GenericEntryBackRightSpeed = SwerveTab.add("BR Speed", 0)
            .withWidget(BuiltInWidgets.kVoltageView)
            .withProperties(Map.of("Min", 0, "Max", 1, "Center", 0, "Orientation", "VERTICAL"))
            .withPosition(14, 5).withSize(2, 5).getEntry();

    static GenericEntry GenericEntryFrontLeftEncoderActual = SwerveEncoders.add("FL Encoder Actual", 0)
            .withWidget(BuiltInWidgets.kTextView).withPosition(0, 0).withSize(2, 1).getEntry();

    static GenericEntry GenericEntryFrontRightEncoderActual = SwerveEncoders.add("FR Encoder Actual", 0)
            .withWidget(BuiltInWidgets.kTextView).withPosition(2, 0).withSize(2, 1).getEntry();

    static GenericEntry GenericEntryBackLeftEncoderActual = SwerveEncoders.add("BL Encoder Actual", 0)
            .withWidget(BuiltInWidgets.kTextView).withPosition(0, 1).withSize(2, 1).getEntry();

    static GenericEntry GenericEntryBackRightEncoderActual = SwerveEncoders.add("BR Encoder Actual", 0)
            .withWidget(BuiltInWidgets.kTextView).withPosition(2, 1).withSize(2, 1).getEntry();

    static GenericEntry GenericEntryFrontLeftEncoderTarget = SwerveEncoders.add("FL Encoder Target", 0)
            .withWidget(BuiltInWidgets.kTextView).withPosition(5, 0).withSize(2, 1).getEntry();

    static GenericEntry GenericEntryFrontRightEncoderTarget = SwerveEncoders.add("FR Encoder Target", 0)
            .withWidget(BuiltInWidgets.kTextView).withPosition(7, 0).withSize(2, 1).getEntry();

    static GenericEntry GenericEntryBackLeftEncoderTarget = SwerveEncoders.add("BL Encoder Target", 0)
            .withWidget(BuiltInWidgets.kTextView).withPosition(5, 1).withSize(2, 1).getEntry();

    static GenericEntry GenericEntryBackRightEncoderTarget = SwerveEncoders.add("BR Encoder Target", 0)
            .withWidget(BuiltInWidgets.kTextView).withPosition(7, 1).withSize(2, 1).getEntry();

    static GenericEntry frontLeftAngle = SwerveEncoders.add("FL Angle", 0).withWidget(BuiltInWidgets.kTextView)
            .withPosition(0, 3).withSize(2, 1).getEntry();
    static GenericEntry frontRightAngle = SwerveEncoders.add("FR Angle", 0)
            .withWidget(BuiltInWidgets.kTextView).withPosition(2, 3).withSize(2, 1).getEntry();
    static GenericEntry backLeftAngle = SwerveEncoders.add("BL Angle", 0).withWidget(BuiltInWidgets.kTextView)
            .withPosition(0, 4).withSize(2, 1).getEntry();
    static GenericEntry backRightAngle = SwerveEncoders.add("BR Angle", 0).withWidget(BuiltInWidgets.kTextView)
            .withPosition(2, 4).withSize(2, 1).getEntry();

    static GenericEntry frontLeft360Angle = SwerveEncoders.add("FL 360 Angle", 0)
            .withWidget(BuiltInWidgets.kTextView).withPosition(4, 3).withSize(2, 1).getEntry();
    static GenericEntry frontRight360Angle = SwerveEncoders.add("FR 360 Angle", 0)
            .withWidget(BuiltInWidgets.kTextView).withPosition(6, 3).withSize(2, 1).getEntry();
    static GenericEntry backLeft360Angle = SwerveEncoders.add("BL 360 Angle", 0)
            .withWidget(BuiltInWidgets.kTextView).withPosition(4, 4).withSize(2, 1).getEntry();
    static GenericEntry backRight360Angle = SwerveEncoders.add("BR 360 Angle", 0)
            .withWidget(BuiltInWidgets.kTextView).withPosition(6, 4).withSize(2, 1).getEntry();

    static GenericEntry ShuffleboardLog = SwerveEncoders.add("ShuffleboardLog", "")
            .withWidget(BuiltInWidgets.kTextView).withSize(4, 2).withPosition(0, 6).getEntry();

    static GenericEntry shuffleboardGyroFused = SwerveTab.add("Gyro - Fused Heading", 0)
            .withWidget(BuiltInWidgets.kTextView).getEntry();

    static GenericEntry shuffleboardDrive=ControlTab.add("Drive Control","Left Stick").getEntry();
    static GenericEntry shuffleboarRotate=ControlTab.add("Swerve Control","Right Stick").getEntry();
    static GenericEntry shuffleboardBallFeedUp=ControlTab.add("Feed Up","").getEntry();
    static GenericEntry shuffleboardBallFeedDown=ControlTab.add("Feed Down","").getEntry();
    static GenericEntry shuffleboardIntakeInD=ControlTab.add("D-Intake In","").getEntry();
    static GenericEntry shuffleboardIntakeOutD=ControlTab.add("D-Intake Out","").getEntry();
    static GenericEntry shuffleboardIntakeInO=ControlTab.add("O-Intake In","").getEntry();
    static GenericEntry shuffleboardIntakeOutO=ControlTab.add("O-Intake Out","").getEntry();
    static GenericEntry shuffleboardIntakeArmD=ControlTab.add("D-Intake Arm","").getEntry();
    static GenericEntry shuffleboardIntakeArmO=ControlTab.add("O-Intake Arm","").getEntry();

    static GenericEntry shuffleboardGyroResetD=ControlTab.add("D-Gyro Reset","").getEntry();
    static GenericEntry shuffleboardGyroResetO=ControlTab.add("O-Gyro Reset","").getEntry();
    static GenericEntry shuffleboardAutoShootD=ControlTab.add("D-Auto Shoot","").getEntry();
    static GenericEntry shuffleboardAutoShootO=ControlTab.add("O-Auto Shoot","").getEntry();
    

    static GenericEntry shuffleboardTurretTurn=ControlTab.add("Turret Turn","").getEntry();
    static GenericEntry shuffleboardClimbLift=ControlTab.add("Climb Lift","").getEntry();
    static GenericEntry shuffleboardClimbLower=ControlTab.add("Climb Down","").getEntry();
   
    static GenericEntry shuffleboardOperatorPivotArm=ControlTab.add("Pivot Arms","").getEntry();
    static GenericEntry shuffleboardOperatorManualShoot=ControlTab.add("Manual Shoot","").getEntry();
   
    static GenericEntry shuffleboardPDPStickyCANFaults=PDPTab.add("Sticky CAN Faults","").getEntry();
    static GenericEntry shuffleboardPDPTotalCurrent=PDPTab.add("Total Current","").getEntry();
    static GenericEntry shuffleboardPDPCurrentC0=PDPTab.add("Current C0","").getEntry();
    static GenericEntry shuffleboardPDPCurrentC1=PDPTab.add("Current C1","").getEntry();
    static GenericEntry shuffleboardPDPCurrentC2=PDPTab.add("Current C2","").getEntry();
    static GenericEntry shuffleboardPDPCurrentC3=PDPTab.add("Current C3","").getEntry();
    static GenericEntry shuffleboardPDPCurrentC4=PDPTab.add("Current C4","").getEntry();
    static GenericEntry shuffleboardPDPCurrentC5=PDPTab.add("Current C5","").getEntry();
    static GenericEntry shuffleboardPDPCurrentC6=PDPTab.add("Current C6","").getEntry();
    static GenericEntry shuffleboardPDPCurrentC7=PDPTab.add("Current C7","").getEntry();
    static GenericEntry shuffleboardPDPCurrentC8=PDPTab.add("Current C8","").getEntry();
    static GenericEntry shuffleboardPDPCurrentC9=PDPTab.add("Current C9","").getEntry();
    static GenericEntry shuffleboardPDPCurrentC10=PDPTab.add("Current C10","").getEntry();  
    static GenericEntry shuffleboardPDPCurrentC11=PDPTab.add("Current C11","").getEntry();
    static GenericEntry shuffleboardPDPCurrentC12=PDPTab.add("Current C12","").getEntry();
    static GenericEntry shuffleboardPDPCurrentC13=PDPTab.add("Current C13","").getEntry();
    static GenericEntry shuffleboardPDPCurrentC14=PDPTab.add("Current C14","").getEntry();
    static GenericEntry shuffleboardPDPCurrentC15=PDPTab.add("Current C15","").getEntry();
    static GenericEntry shuffleboardPDPCurrentC16=PDPTab.add("Current C16","").getEntry();
    static GenericEntry shuffleboardPDPCurrentC17=PDPTab.add("Current C17","").getEntry();
    static GenericEntry shuffleboardPDPCurrentC18=PDPTab.add("Current C18","").getEntry();
    static GenericEntry shuffleboardPDPCurrentC19=PDPTab.add("Current C19","").getEntry();
    static GenericEntry shuffleboardPDPCurrentC20=PDPTab.add("Current C20","").getEntry();
    static GenericEntry shuffleboardPDPCurrentC21=PDPTab.add("Current C21","").getEntry();
    static GenericEntry shuffleboardPDPCurrentC22=PDPTab.add("Current C22","").getEntry();
    static GenericEntry shuffleboardPDPCurrentC23=PDPTab.add("Current C23","").getEntry();
    
    static GenericEntry shuffleboardTurretPos=ShooterTab.add("Turret Position","").getEntry();
    
    static GenericEntry shuffleboardLeftLimit=ShooterTab.add("Left Limit","").getEntry();
    static GenericEntry shuffleboardRightLimit=ShooterTab.add("Right Limit","").getEntry();
    static GenericEntry shuffleboardShooterTop=ShooterTab.add("Top Speed",Constants.AutoModes.AutoShotTopSpeed).getEntry();
    static GenericEntry shuffleboardShooterBottom=ShooterTab.add("Bottom Speed",Constants.AutoModes.AutoShotBottomSpeed).getEntry();

    static GenericEntry shuffleboardShooterTopVel=ShooterTab.add("Top Velocity","").getEntry();
    static GenericEntry shuffleboardShooterBottomVel=ShooterTab.add("Bottom Velocity","").getEntry();
    static GenericEntry shuffleobardShooterMultipler=ShooterTab.add("Shooter Adjustment",1.0)
                                .getEntry();

    static GenericEntry shuffleobardLimelightAdj=ShooterTab.add("Limelight Adjustment",Constants.LimeLightValues.kVisionXOffset)
                                .getEntry();

    static GenericEntry shuffleboardRobotMoving=SwerveTab.add("Robot Moving","")
                               .getEntry();
 
    static String ShuffleboardLogString;
    // #endregion
    // #endregion

    // 
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
     * Usethis method to define your button->command mappings. Buttons can be
     * created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
     * it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        
        //FIXIt when done getting shooter values
        double topSpeed=shuffleboardShooterTop.getDouble(0);
        double bottomSpeed=shuffleboardShooterBottom.getDouble(0);
        //KK SIMPLE VERSION DISABLED
        //final ParallelCommandGroup LoadandShootCommand = new ParallelCommandGroup(new ShooterCommand(m_Shooter,m_Limelight,topSpeed,bottomSpeed,true),
         //       new ConveyorCommand(m_Conveyor,Constants.conveyorUpSpeed),
        //       new FeederCommand(m_Feeder,Constants.FeederSpeed));
        
        
      final IntakeArmCommand intakeArmCommand = new IntakeArmCommand(m_Pnuematics);
        final ClimbLiftCommand climbLiftCommand = new ClimbLiftCommand(m_Pnuematics);
     
        final GyroResetCommand gyroResetCommand = new GyroResetCommand(m_RobotDrive,m_Limelight);
        
       
        SpectrumAxisButton operatorAutoShoot = new SpectrumAxisButton(m_xBoxOperator,ButtonConstants.OperatorAutoShootAxis ,ButtonConstants.TriggerThreshold,SpectrumAxisButton.ThresholdType.GREATER_THAN);
        shuffleboardAutoShootO.setString("Right Trigger");

        SpectrumAxisButton operatorTurretOveride= new SpectrumAxisButton(m_xBoxOperator,ButtonConstants.OperatorOverrideAxis ,ButtonConstants.TriggerThreshold,SpectrumAxisButton.ThresholdType.GREATER_THAN);
       
        JoystickButton driverIntakeIn= new JoystickButton(m_xBoxDriver, ButtonConstants.DriverIntakeIn);
        shuffleboardIntakeInD.setString("D-Button " + ButtonConstants.DriverIntakeIn);

        JoystickButton operatorIntakeIn= new JoystickButton(m_xBoxOperator, ButtonConstants.OperatorIntakeIn );
        shuffleboardIntakeInO.setString("O-Button " + ButtonConstants.OperatorIntakeIn);

        JoystickButton operatorClimbArm = new JoystickButton(m_xBoxOperator, ButtonConstants.OperatorClimbLift );
       

        JoystickButton driverIntakeOut= new JoystickButton(m_xBoxDriver, ButtonConstants.DriverIntakeOut);
        shuffleboardIntakeOutD.setString("D-Button " + ButtonConstants.DriverIntakeOut);

        JoystickButton operatorIntakeOut= new JoystickButton(m_xBoxOperator, ButtonConstants.OperatorIntakeOut );
        shuffleboardIntakeOutO.setString("O-Button " + ButtonConstants.OperatorIntakeOut);

        JoystickButton driverIntakeArm = new JoystickButton(m_xBoxDriver,ButtonConstants.DriverIntakeArm);
        shuffleboardIntakeArmD.setString("D-Button" + ButtonConstants.DriverIntakeArm);
        JoystickButton operatorIntakeArm = new JoystickButton(m_xBoxOperator,ButtonConstants.OperatorIntakeArm);
        shuffleboardIntakeArmO.setString("O-Button" + ButtonConstants.OperatorIntakeArm);
        
        JoystickButton driverGyroReset = new JoystickButton(m_xBoxDriver,ButtonConstants.DriverGyroReset);
        shuffleboardGyroResetD.setString("D-Button " + Constants.ButtonConstants.DriverGyroReset);       
        JoystickButton driverGyroReset2 = new JoystickButton(m_xBoxDriver,ButtonConstants.DriverGyroReset2);
        JoystickButton operatorGyroReset = new JoystickButton(m_xBoxOperator,ButtonConstants.OperatorGyroReset);
        shuffleboardGyroResetO.setString("O-Button " + Constants.ButtonConstants.OperatorGyroReset);
        JoystickButton operatorGyroReset2 = new JoystickButton(m_xBoxOperator,ButtonConstants.OperatorGyroReset2);
       
        POVButton driverTurretLeftButton=new POVButton(m_xBoxDriver,ButtonConstants.TurretLeftPOV);
        POVButton driverTurretRightButton=new POVButton(m_xBoxDriver,ButtonConstants.TurretRightPOV); 
       
       
        SpectrumAxisButton operatorTurretLeft = new SpectrumAxisButton(m_xBoxOperator,ButtonConstants.OperatorTurretAxis,ButtonConstants.JoystickLeftThreshold,SpectrumAxisButton.ThresholdType.GREATER_THAN);
        SpectrumAxisButton operatorTurretRight = new SpectrumAxisButton(m_xBoxOperator,ButtonConstants.OperatorTurretAxis,ButtonConstants.JoystickRightThreshold,SpectrumAxisButton.ThresholdType.LESS_THAN);
        SpectrumAxisButton operatorTurretAutoFind = new SpectrumAxisButton(m_xBoxOperator,ButtonConstants.OperatorTurretFindAxis,ButtonConstants.JoystickUpThreshold,SpectrumAxisButton.ThresholdType.GREATER_THAN);
        SpectrumAxisButton operatorTurretAutoFindStop = new SpectrumAxisButton(m_xBoxOperator,ButtonConstants.OperatorTurretFindAxis,ButtonConstants.JoystickDownThreshold,SpectrumAxisButton.ThresholdType.LESS_THAN);    

        SpectrumAxisButton operatorClimbUp = new SpectrumAxisButton(m_xBoxOperator,ButtonConstants.OperatorClimbAxis,ButtonConstants.JoystickUpThreshold,SpectrumAxisButton.ThresholdType.LESS_THAN);
        SpectrumAxisButton operatorClimbDown = new SpectrumAxisButton(m_xBoxOperator,ButtonConstants.OperatorClimbAxis,ButtonConstants.JoystickDownThreshold,SpectrumAxisButton.ThresholdType.GREATER_THAN);
       
        JoystickButton operatorPivotArm = new JoystickButton(m_xBoxOperator,ButtonConstants.OperatorPivotArm);
        shuffleboardOperatorPivotArm.setString("O-Button " + Constants.ButtonConstants.OperatorPivotArm);
       
      //  JoystickButton operatorManualShoot =new JoystickButton(m_xBoxOperator,ButtonConstants.OperatorShootManual);
      //  shuffleboardOperatorManualShoot.setString("O-Button " + Constants.ButtonConstants.OperatorShootManual);
        JoystickButton operatorAutoFind =new JoystickButton(m_xBoxOperator,ButtonConstants.OperatorAutoTurretMode);
         

        POVButton operatorShoot1Button=new POVButton(m_xBoxOperator,ButtonConstants.OperatorShooter1POV);
        POVButton operatorShoot2Button=new POVButton(m_xBoxOperator,ButtonConstants.OperatorShooter2POV); 
        POVButton operatorShoot3Button=new POVButton(m_xBoxOperator,ButtonConstants.OperatorShooter3POV);
        POVButton operatorShoot4Button=new POVButton(m_xBoxOperator,ButtonConstants.OperatorShooter4POV); 
       
        //turretAutoFind.whenHeld(turretAutoCommand);
        
         
        driverGyroReset.whenPressed(gyroResetCommand);
        driverGyroReset2.whenPressed(gyroResetCommand);
        operatorGyroReset.whenPressed(gyroResetCommand);
        operatorGyroReset2.whenPressed(gyroResetCommand);

     
   }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand(Integer selectedMode) {
        Command autoCommand = new AutoDoNothingCommand(); // Default Command is DoNothing
        System.out.println("Autonomouse Selected Mode = " + selectedMode);
        autoCommand = new AutoDoNothingCommand();

        return autoCommand;
    }   
    public void refreshSmartDashboard()
    {  
        frontLeftAngle.setDouble(m_RobotDrive.getFrontLeftAngle());
        frontRightAngle.setDouble(m_RobotDrive.getFrontRightAngle());
        backLeftAngle.setDouble(m_RobotDrive.getBackLeftAngle());
        backRightAngle.setDouble(m_RobotDrive.getbackRightAngle());
        m_Limelight.update();
        //KK REplacing on 4/15
        /*
        if(m_Limelight.isTargetAvailible()){              
                GenericEntryVisionDistance.setDouble(m_Limelight.getDistance());                
                if((m_Limelight.getDistance()>=Constants.shooterSweetSpotLow) && (m_Limelight.getDistance()<=Constants.shooterSweetSpotHigh)){
                        m_LEDMode=LEDMode.ONTARGETSWEET;                       
                }else{
                        m_LEDMode=LEDMode.ONTARGET;
                }
        }else {
               m_LEDMode=LEDMode.OFFTARGET;
                GenericEntryVisionDistance.setDouble(0);
        }
        */
        if(m_Limelight.isOnTargetX()){
                GenericEntryVisionDistance.setDouble(m_Limelight.getDistance());                
                if((m_Limelight.getDistance()>=Constants.shooterSweetSpotLow) && (m_Limelight.getDistance()<=Constants.shooterSweetSpotHigh)){
                        m_LEDMode=LEDMode.ONTARGETSWEET;                       
                }else{
                        m_LEDMode=LEDMode.ONTARGET;
                }
        }else{
                if(m_Limelight.isTargetAvailible()){              
                        GenericEntryVisionDistance.setDouble(m_Limelight.getDistance());                
                        if((m_Limelight.getDistance()>=Constants.shooterSweetSpotLow) && (m_Limelight.getDistance()<=Constants.shooterSweetSpotHigh)){
                                m_LEDMode=LEDMode.OFFTARGETSWEET;                       
                        }else{
                                m_LEDMode=LEDMode.ONTARGET;
                        }
                }else {
                        m_LEDMode=LEDMode.OFFTARGET;
                        GenericEntryVisionDistance.setDouble(0);
                }
                
        }
        //END or Replacement
      //  shuffleboardShooterTopVel.setDouble(m_Shooter.getTopMotorVelocity());
      //  shuffleboardShooterBottomVel.setDouble(m_Shooter.getBottomMotorVelocity());
        //if feeder is spinning at target speed
       // if(m_Feeder.getFeederSpeed()<Constants.FeederThresholdForLED){
             
       //         m_LEDMode=LEDMode.SHOOTING;
       // }
      //  shuffleboardTurretPos.setString(" " + m_turret.getPosition());
      //  shuffleboardLeftLimit.setBoolean(m_turret.hitLeftLimit());
      //  shuffleboardRightLimit.setBoolean(m_turret.hitRightLimit());
      //  System.out.println("Physical Turret - "+  m_turret.hitRightPhysicalLimit() + " -- "+ m_turret.getPosition());
        m_Limelight.setOffSet((shuffleobardLimelightAdj.getDouble(Constants.LimeLightValues.kVisionXOffset)));
        LEDUpdate();
        updateRobotMoving();
      
}
    
     private void updateRobotMoving(){
            
        }

     public void disabledPerioidicUpdates(){
   //      shuffleboardLeftLimit.setBoolean(m_turret.hitLeftLimit());
   //     shuffleboardRightLimit.setBoolean(m_turret.hitRightLimit());
        frontLeftAngle.setDouble(m_RobotDrive.getFrontLeftAngle());
        frontRightAngle.setDouble(m_RobotDrive.getFrontRightAngle());
        backLeftAngle.setDouble(m_RobotDrive.getBackLeftAngle());
        backRightAngle.setDouble(m_RobotDrive.getbackRightAngle());
             LEDUpdate();
        updateRobotMoving();

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
    private void LEDUpdate(){            
    /*
        if(m_LEDMode!=m_oldLEDmode){            
                if(m_LEDMode==LEDMode.ONTARGET){
                        m_ledStrip.setColor(Colors.YELLOW);
                        m_ledStrip.setMode(LEDMODE_SOLID);
                        m_ledFlash=false;
                }
                if(m_LEDMode==LEDMode.ONTARGETSWEET){
                        m_ledStrip.setColor(Colors.PURPLE);
                        m_ledStrip.setMode(LEDMODE_SOLID);
                        m_ledFlash=true;
                }
                if(m_LEDMode==LEDMode.OFFTARGETSWEET){
                        m_ledStrip.setColor(Colors.PURPLE);
                        m_ledStrip.setMode(LEDMODE_SOLID);
                        m_ledFlash=false;
                }
                if(m_LEDMode==LEDMode.OFFTARGET){
                        m_ledStrip.setColor(Colors.RED);
                        m_ledStrip.setMode(LEDMODE_SOLID);
                        m_ledFlash=false;
                }
                if(m_LEDMode==LEDMode.SHOOTING){
                        m_ledStrip.setColor(Colors.GREEN);
                        m_ledStrip.setMode(LEDMODE_SOLID);
                        m_ledFlash=false;
                }
                if(m_LEDMode==LEDMode.CLIMBING){
                        m_ledStrip.setColor(Colors.ORANGE);
                        m_ledStrip.setMode(LEDMODE_SOLID);
                        m_ledFlash=false;
                }
                if(m_LEDMode==LEDMode.AUTOMODE){
                        m_ledStrip.setColor(Colors.PURPLE);
                        m_ledStrip.setMode(LEDMODE_RAINBOW);
                        m_ledFlash=false;
                }
                if(m_LEDMode==LEDMode.DISBLED){
                        m_ledStrip.setMode(LEDMODE_WAVE);
                        m_ledStrip.setColor(Colors.PURPLE);
                        m_ledFlash=false;
                }
                if(m_LEDMode==LEDMode.TELEOP){
                        m_ledStrip.setMode(LEDMODE_WAVE);
                        m_ledStrip.setColor(Colors.PINK);
                        m_ledFlash=false;
                }
        }
        if(m_ledFlash){
                m_ledFlashDelayCount++;
                if(m_ledFlashMode==false){
                        //(m_ledFlashDelayCouunt>10)
                        if(m_ledFlashDelayCount>Constants.LEDS.FLASH_DELAY) {
                                m_ledStrip.setMode(LEDMODE_OFF);
                                m_ledFlashMode=true;
                                m_ledStrip.update();
                                m_ledFlashDelayCount=0;
                        }
                } else{
                        if(m_ledFlashDelayCount>Constants.LEDS.FLASH_DELAY) {                       
                                m_ledStrip.setMode(LEDMODE_SOLID);                        
                                m_ledFlashMode=false;
                                m_ledStrip.update();
                                m_ledFlashDelayCount=0;
                        }
                }

        } else {
                m_ledStrip.update();
       
        }
        m_oldLEDmode=m_LEDMode;
     */       
    }
  
    public void LEDAutoMode(){
        m_LEDMode=LEDMode.AUTOMODE;
        LEDUpdate();
        }
    public void TeleopMode(){
        m_LEDMode=LEDMode.TELEOP;      
        LEDUpdate();
        
}
    public void DisableMode(){
            m_LEDMode=LEDMode.DISBLED;
            LEDUpdate();
    }
}
