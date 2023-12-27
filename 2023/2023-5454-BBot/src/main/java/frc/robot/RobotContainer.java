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
import frc.robot.Constants.LEDS.Colors;
import frc.robot.classes.Limelight;
import frc.robot.classes.LEDStrip;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.common.control.PidController;
import frc.robot.classes.PhotonVision;
import frc.robot.commands.*;

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
//     private NavX m_NavX = new NavX(SPI.Port.kMXP);
   // private final DriveSubsystem m_RobotDrive = new DriveSubsystem(m_ahrs);
  // private final DrivetrainSubsystem m_RobotDrive = new DrivetrainSubsystem(m_NavX);
   private final SwerveSubsystem m_RobotDrive = new SwerveSubsystem();
   private final RotateArmSubsystem m_RotateArm = new RotateArmSubsystem(Constants.RotateArm.rotateMotorPort,Constants.RotateArm.absEncoderPort);
   private final PneumaticsSubsystem m_Pneumatics = new PneumaticsSubsystem(Constants.Pneumatics.CompressorID,Constants.Pneumatics.clawSolenoid1ID
                                                ,Constants.Pneumatics.clawSolenoid2ID,Constants.Pneumatics.extensionSolenoidID,Constants.Pneumatics.holdSolenoidID);

    private final Limelight m_Limelight = new Limelight(Constants.LimeLightValues.targetHeight, Constants.LimeLightValues.limelightHeight, Constants.LimeLightValues.limelightAngle,Constants.LimeLightValues.kVisionXOffset,80);
     
     private PhotonVision m_PhotonVision = new PhotonVision();

     private final LEDStrip m_ledStrip = new LEDStrip(Constants.LEDS.PORT, Constants.LEDS.COUNT);
     private static enum LEDMode
     {
                     NOTSET,DISBLED, AUTOMODE, OFFTARGET, OFFTARGETSWEET, ONTARGETSWEET,ONTARGET,SHOOTING,CLIMBING,TELEOP;	
     }
     private LEDMode m_LEDMode=LEDMode.DISBLED;
     private boolean m_disabled=true;
     private boolean m_ledFlash=false;
     private boolean m_ledFlashMode=false;
     private int m_ledFlashDelayCount=0;
     private static final int LEDMODE_WAVE = 0;
     private static final int LEDMODE_BAR = 1;
     private static final int LEDMODE_RAINBOW = 2;
     private static final int LEDMODE_SOLID = 3;
     private static final int LEDMODE_OFF = 4;
     private LEDMode m_oldLEDmode=LEDMode.NOTSET; 
     
                                                  
    // #region Shuffleboard


    private final PowerDistribution m_robotPDH = new PowerDistribution(1, PowerDistribution.ModuleType.kRev);



    // #region Create Shuffleboard Tabs
    private static ShuffleboardTab SwerveTab = Shuffleboard.getTab("Swerve");
    private static ShuffleboardTab SwerveEncoders = Shuffleboard.getTab("SwerveEncoders");
    private static ShuffleboardTab AutoTab = Shuffleboard.getTab("Auto");
    private static ShuffleboardTab ShooterTab=Shuffleboard.getTab("Shooter");
    private static ShuffleboardTab ControlTab = Shuffleboard.getTab("Controls");
    private static ShuffleboardTab PDPTab = Shuffleboard.getTab("PDP");
    private static ShuffleboardTab EncoderTab = Shuffleboard.getTab("EncoderValues");
    // #endregion
    
   
    // #region NetworkEntries
    // Create Network Table Entries

   
    static GenericEntry networkTableEntryVisionDistance = ShooterTab.add("Vision Distance", 0)
            .withWidget(BuiltInWidgets.kNumberBar).withSize(2, 2).getEntry();

    
    static GenericEntry networkTableEntryFrontLeftSpeed = SwerveTab.add("FL Speed", 0)
            .withWidget(BuiltInWidgets.kVoltageView)
            .withProperties(Map.of("Min", 0, "Max", 1, "Center", 0, "Orientation", "VERTICAL"))
            .withPosition(1, 0).withSize(2, 5).getEntry();

    static GenericEntry networkTableEntryFrontRightSpeed = SwerveTab.add("FR Speed", 0)
            .withWidget(BuiltInWidgets.kVoltageView)
            .withProperties(Map.of("Min", 0, "Max", 1, "Center", 0, "Orientation", "VERTICAL"))
            .withPosition(14, 0).withSize(2, 5).getEntry();

    static GenericEntry networkTableEntryBackLeftSpeed = SwerveTab.add("BL Speed", 0)
            .withWidget(BuiltInWidgets.kVoltageView)
            .withProperties(Map.of("Min", 0, "Max", 1, "Center", 0, "Orientation", "VERTICAL"))
            .withPosition(1, 5).withSize(2, 5).getEntry();

    static GenericEntry networkTableEntryBackRightSpeed = SwerveTab.add("BR Speed", 0)
            .withWidget(BuiltInWidgets.kVoltageView)
            .withProperties(Map.of("Min", 0, "Max", 1, "Center", 0, "Orientation", "VERTICAL"))
            .withPosition(14, 5).withSize(2, 5).getEntry();

    static GenericEntry networkTableEntryFrontLeftEncoderActual = SwerveEncoders.add("FL Encoder Actual", 0)
            .withWidget(BuiltInWidgets.kTextView).withPosition(0, 0).withSize(2, 1).getEntry();
 
    static GenericEntry networkTableEntryFrontRightEncoderActual = SwerveEncoders.add("FR Encoder Actual", 0)
            .withWidget(BuiltInWidgets.kTextView).withPosition(2, 0).withSize(2, 1).getEntry();

    static GenericEntry networkTableEntryBackLeftEncoderActual = SwerveEncoders.add("BL Encoder Actual", 0)
            .withWidget(BuiltInWidgets.kTextView).withPosition(0, 1).withSize(2, 1).getEntry();

    static GenericEntry networkTableEntryBackRightEncoderActual = SwerveEncoders.add("BR Encoder Actual", 0)
            .withWidget(BuiltInWidgets.kTextView).withPosition(2, 1).withSize(2, 1).getEntry();

    static GenericEntry networkTableEntryFrontLeftEncoderTarget = SwerveEncoders.add("FL Encoder Target", 0)
            .withWidget(BuiltInWidgets.kTextView).withPosition(5, 0).withSize(2, 1).getEntry();

    static GenericEntry networkTableEntryFrontRightEncoderTarget = SwerveEncoders.add("FR Encoder Target", 0)
            .withWidget(BuiltInWidgets.kTextView).withPosition(7, 0).withSize(2, 1).getEntry();

    static GenericEntry networkTableEntryBackLeftEncoderTarget = SwerveEncoders.add("BL Encoder Target", 0)
            .withWidget(BuiltInWidgets.kTextView).withPosition(5, 1).withSize(2, 1).getEntry();

    static GenericEntry networkTableEntryBackRightEncoderTarget = SwerveEncoders.add("BR Encoder Target", 0)
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
   
    private boolean m_turretHasReset =false;
    private RotateArmCommand rotateArm= new RotateArmCommand(m_RotateArm,() -> (m_xBoxOperator.getLeftX()));

    private SequentialCommandGroup autoScoreMoveBWD = new SequentialCommandGroup(new AutoMoveCommand(m_RobotDrive,180), new zMoveArmExtendABS(m_RotateArm,m_Pneumatics,m_Limelight,Constants.TargetHeight.MIDDLECONE,true,true),
                                                        new zMoveArmRetractABS(m_RotateArm, m_Pneumatics), new ClawOpenCloseCommand(m_Pneumatics));

    private SetPositionCommand setRestingPosCommand = new SetPositionCommand(m_RotateArm, m_Pneumatics,Constants.SETPOSPOSITIONS.STARTING);
    private SetPositionCommand setPlayerPosCommand = new SetPositionCommand(m_RotateArm, m_Pneumatics,Constants.SETPOSPOSITIONS.PLAYERSTATION);                                                  
    private SetPositionCommand setScorePosCommand = new SetPositionCommand(m_RotateArm, m_Pneumatics,Constants.SETPOSPOSITIONS.SCORE);                                                  
    private HoldArmSwapCommand swapHoldArm = new HoldArmSwapCommand(m_Pneumatics);
    /**
    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // Configure the button bindings
        configureButtonBindings();

     m_RobotDrive.setDefaultCommand(
                new DefaultDriveCommand(m_RobotDrive,
                        () -> m_xBoxDriver.getRightX(),
                        () -> m_xBoxDriver.getLeftX(),
                        () -> m_xBoxDriver.getLeftY()));

    }

    private void createAutoCommands(){
        
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

        final GyroResetCommand gyroResetCommand = new GyroResetCommand(m_RobotDrive,m_Limelight);

        Trigger operatorRotate = new Trigger(() -> Math.abs(m_xBoxOperator.getLeftX())>ButtonConstants.RotateDeadBand);
        operatorRotate.whileTrue(rotateArm);
        Command clawOpenClose = new ClawOpenCloseCommand(m_Pneumatics);
        Command topClawOpenClose = new OpenTopClawCommand(m_Pneumatics);
        Command bottomClawOpen= new OpenBottomClawCommand(m_Pneumatics);
        Command clawExtend=new ExtendClawCommand(m_Pneumatics);
        Trigger fullClaw = new JoystickButton(m_xBoxOperator,1);
        Trigger topClaw = new JoystickButton(m_xBoxOperator,2);
        Trigger extendClaw  = new JoystickButton(m_xBoxOperator,3);
        Trigger bottomClaw = new JoystickButton(m_xBoxOperator,4);
        
        fullClaw.toggleOnTrue(clawOpenClose);
        topClaw.toggleOnTrue(topClawOpenClose);
        extendClaw.toggleOnTrue(clawExtend);
        bottomClaw.toggleOnTrue(bottomClawOpen);
        Trigger restingPositionAuto = new Trigger(() -> Math.abs(m_xBoxOperator.getRawAxis(2))>Constants.ButtonConstants.RotateDeadBand);
        restingPositionAuto.toggleOnTrue(setRestingPosCommand);

        Trigger playerStationAuto = new JoystickButton(m_xBoxOperator,5);
        playerStationAuto.toggleOnTrue(setPlayerPosCommand);

        Trigger playerScoreAuto = new JoystickButton(m_xBoxOperator,6);
        playerScoreAuto.toggleOnTrue(setScorePosCommand);
        
        Trigger swapArmHold = new JoystickButton(m_xBoxOperator,7);
        swapArmHold.toggleOnTrue(swapHoldArm);
        
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
          case Constants.AutoModes.autoMoveForward:
      //     autoCommand = new SequentialCommandGroup(new AutoMoveCommand(m_RobotDrive,0,5),new AutoMoveCommand(m_RobotDrive,180,10));   
          default:
          autoCommand = new SequentialCommandGroup(new AutoMoveCommand(m_RobotDrive,180,0.7),new AutoMoveCommand(m_RobotDrive,0,1.8));   
          //autoCommand = new AutoDoNothingCommand();
        }
        return autoCommand;
        // return m_autoCommand;
    }   
    public void refreshSmartDashboard()
    {  
        frontLeftAngle.setDouble(m_RobotDrive.getFrontLeftAngle());
        frontRightAngle.setDouble(m_RobotDrive.getFrontRightAngle());
        backLeftAngle.setDouble(m_RobotDrive.getBackLeftAngle());
        backRightAngle.setDouble(m_RobotDrive.getBackRightAngle());
        m_RobotDrive.currentAngle();
        m_Limelight.update();
        //override disabled led mode
        if(m_disabled){
                m_LEDMode=LEDMode.DISBLED;
        }
        LEDUpdate();
        updateRobotMoving();
      
}
    
     private void updateRobotMoving(){
                /*if (m_RobotDrive.IsRobotMoving()){
                        shuffleboardRobotMoving.setString("True");
                }
             System.out.println("dri   else{
                        shuffleboardRobotMoving.setString("False");     
                }*/
        }

     public void disabledPerioidicUpdates(){
        LEDUpdate();
        updateRobotMoving();

    }

    public void disableLimelights(){
            m_Limelight.turnLEDOff();
    }
    public void enableLimelights(){
            m_Limelight.turnLEDOn();
            m_disabled=false;
    }
    public void resetDriveModes(){
//        m_RobotDriveeDriveMode();
    }
    private void LEDUpdate(){            
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
                        m_ledStrip.setColor(Colors.BLUE);
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
            m_disabled=true;
            m_LEDMode=LEDMode.DISBLED;
            LEDUpdate();
    }
    public void EnableMode(){
      m_disabled=false;
}

}
