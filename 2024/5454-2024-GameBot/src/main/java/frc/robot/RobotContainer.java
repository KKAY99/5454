// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.sql.DriverPropertyInfo;
import java.util.List;
import javax.swing.JToggleButton;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.XboxController.Axis;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants.InputControllers;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.LEDConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.TurretConstants;
import frc.robot.Constants.BlinkinConstants.LEDStates;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.BlinkinConstants;
import frc.robot.Constants.ButtonBindings;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.utilities.ADABreakBeam;
import frc.robot.utilities.AutoCommands;
import frc.robot.utilities.AutoPose2D;
import frc.robot.utilities.Blinkin;
import frc.robot.utilities.ObsidianCANSparkMax;
import frc.robot.utilities.ObsidianCANSparkMax;
import frc.robot.utilities.LED;
import frc.robot.utilities.Limelight;
import frc.robot.utilities.ChooseYourOwnAdventureAuto;

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
    //STANDARD DRIVING
    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;
    //DRONE DRIVING
    //private final int translationAxis = XboxController.Axis.kRightY.value;
    //private final int strafeAxis = XboxController.Axis.kRightX.value;
    //private final int rotationAxis = XboxController.Axis.kLeftX.value;
    
    private final int rightTriggerAxis = XboxController.Axis.kRightTrigger.value;
    private final int leftTriggerAxis = XboxController.Axis.kLeftTrigger.value;
 
    private XboxController m_xBoxDriver = new XboxController(InputControllers.kXboxDrive);
    private XboxController m_xBoxOperator = new XboxController(InputControllers.kXboxOperator);
    private Joystick m_CustomController = new Joystick(InputControllers.kCustomController);
    
    private SendableChooser<String> m_autoChooser = new SendableChooser<>(); 
    private SendableChooser<AutoConstants.StartingLocations> m_autoStart = new SendableChooser<>(); 
    private SendableChooser<Double> m_autoDelay = new SendableChooser<>(); 
    private SendableChooser<Boolean> m_shootFinalNote = new SendableChooser<>();
    private SendableChooser<Boolean> m_shouldUseDashBoardValues = new SendableChooser<>();
    private SendableChooser<AutoPose2D> m_autoPath1 = new SendableChooser<>();
    private SendableChooser<AutoPose2D> m_autoPath2 = new SendableChooser<>(); 
    private SendableChooser<AutoPose2D> m_autoPath3 = new SendableChooser<>(); 
    private SendableChooser<AutoPose2D> m_autoPath4 = new SendableChooser<>(); 
    private SendableChooser<AutoPose2D> m_autoPath5 = new SendableChooser<>(); 
   
    private Integer m_hasNoteLoopCounter=0;
    private Integer khasNoteLoopLEDLimit=5;

    private Swerve m_swerve = new Swerve(new SwerveIO(){});
//    private LED m_led=new LED(Constants.LEDConstants.blinkInPWM,Constants.LEDConstants.ledPWM,Constants.LEDConstants.ledCount);
    //private LED m_led= new LED(Constants.LEDConstants.ledPWM,Constants.LEDConstants.ledCount);
    private Blinkin m_blinkin=new Blinkin(Constants.BlinkinConstants.pwmID);
    private LED m_candle=new LED(LEDConstants.candleId,Constants.CAN.CanivoreBus,LEDConstants.candleLEDCount);
    private DigitalInput m_brakeButton = new DigitalInput(Constants.brakeButton);
    private Limelight m_TurretLimelight = new Limelight(Constants.LimeLightValues.targetHeight,Constants.LimeLightValues.limelightTurretHeight,
                                        Constants.LimeLightValues.limelightTurretAngle);
    /*private Limelight m_StaticLimelight = new Limelight(Constants.LimeLightValues.targetHeight,Constants.LimeLightValues.limelightTurretHeight,
                                        Constants.LimeLightValues.limelightTurretAngle,0,0,Constants.LimeLightValues.staticLimelightName);*/
    private NoteFlipperSubsystem m_flip=new NoteFlipperSubsystem(Constants.NoteFlipConstants.canID);
    private TurretSubsystem m_turret=new TurretSubsystem(Constants.TurretConstants.turretMotorPort,
                                         Constants.TurretConstants.turretLimitSwitchPort,
                                         m_TurretLimelight,m_swerve.getYSpeedSupplier());
    private IntakeSubsystem m_intake=new IntakeSubsystem(m_turret,Constants.IntakeConstants.intakeMotorPort1,
                                         Constants.IntakeConstants.intakeMotorPort2,
                                         Constants.IntakeConstants.intakeExtensionPort,
                                         Constants.IntakeConstants.intakeBreakBeamport);
    //private ClimbSubsystem m_climb = new ClimbSubsystem(Constants.climbConstants.climbPort);
    private ShooterSubsystem m_shooter=new ShooterSubsystem(m_TurretLimelight,Constants.ShooterConstants.shooterMotorPort1,
                                           Constants.ShooterConstants.shooterMotorPort2,Constants.ShooterConstants.shooterAnglePort,
                                           Constants.ShooterConstants.feederMotorPort,Constants.ShooterConstants.encoderCanID,
                                           Constants.ShooterConstants.baseMotorSpeed);

    private boolean m_isBrakeButtonToggled=false;
    private boolean m_brakeButtonPressed=false;
    private boolean m_HasHomed=false;

    public RobotContainer(){
      //Named Commands
      NamedCommands.registerCommand("autoscore",new AutoDoNothingCommand()); 
      // Configure the button bindings
        configureButtonBindings();
        //Create Auto Commands
        createAutonomousCommandList(); 
        
    }
       
    
    /*m*
     * Use this method to define your button->command mappings. Buttons can be
     * created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
     * it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings(){
     // IntakeToggleCommand intakeToggleTrueIn=new IntakeToggleCommand(m_intake,Constants.IntakeConstants.intakeSpeed);
     // JoystickButton intakeToggleTrueButtonIn=new JoystickButton(m_xBoxDriver,ButtonBindings.driverintakeToggleButtonIn);
     // intakeToggleTrueButtonIn.onTrue(intakeToggleTrueIn);

      
      //IntakeToggleCommand intakeConveyIn=new IntakeToggleCommand(m_intake,Constants.IntakeConstants.intakeSpeed,false);
      //intake smart 
      IntakeToggleCommand intakeConveyIn=new IntakeToggleCommand(m_intake,Constants.IntakeConstants.intakeSpeed,false);
      JoystickButton intakeToggleTrueButtonIn=new JoystickButton(m_xBoxDriver,ButtonBindings.driverintakeToggleButtonIn);
      intakeToggleTrueButtonIn.whileTrue(intakeConveyIn);

      IntakeToggleCommand intakeToggleOperatorTrueIn=new IntakeToggleCommand(m_intake,Constants.IntakeConstants.intakeSpeed,false);
      JoystickButton intakeToggleTrueOperatorButtonIn=new JoystickButton(m_xBoxOperator,ButtonBindings.operatorintakeToggleButtonIn);
      intakeToggleTrueOperatorButtonIn.whileTrue(intakeToggleOperatorTrueIn);

      IntakeToggleCommand intakeToggleTrueOut=new IntakeToggleCommand(m_intake,-Constants.IntakeConstants.intakeSpeed,false);
      JoystickButton intakeToggleTrueButtonOut=new JoystickButton(m_xBoxDriver,ButtonBindings.driverintakeToggleButtonOut);
      intakeToggleTrueButtonOut.whileTrue(intakeToggleTrueOut);

      IntakeToggleCommand intakeToggleOperatorTrueOut=new IntakeToggleCommand(m_intake,-Constants.IntakeConstants.intakeSpeed,false);
      JoystickButton intakeToggleOperatorTrueButtonOut=new JoystickButton(m_xBoxOperator,ButtonBindings.operatorintakeToggleButtonOut);
      intakeToggleOperatorTrueButtonOut.whileTrue(intakeToggleOperatorTrueOut);

      IntakeConveyCommand intakeConvey=new IntakeConveyCommand(m_intake,m_shooter,Constants.IntakeConstants.intakeSpeed);
      JoystickButton intakeConveyButton=new JoystickButton(m_xBoxOperator,ButtonBindings.operatorintakeConveyButtonIn);
      intakeConveyButton.onTrue(intakeConvey);

      IntakeConveyCommand intakeConveyDriv=new IntakeConveyCommand(m_intake,m_shooter,Constants.IntakeConstants.intakeSpeed);
      JoystickButton intakeConveyButtonDriv=new JoystickButton(m_xBoxDriver,ButtonBindings.operatorintakeConveyButtonIn);
      intakeConveyButtonDriv.onTrue(intakeConveyDriv);

      /*TurretCommand turretLeft=new TurretCommand(m_turret,Constants.TurretConstants.turretSpeed);
      POVButton turretLeftButton=new POVButton(m_xBoxDriver,Constants.ButtonBindings.driverturretPOVLeft);
      turretLeftButton.whileTrue(turretLeft);*/

      TurretCommand turretOperatorLeft=new TurretCommand(m_turret,Constants.TurretConstants.turretSpeed);
      POVButton turretOperatorLeftButton=new POVButton(m_xBoxOperator,Constants.ButtonBindings.operatorturretPOVLeft);
      turretOperatorLeftButton.whileTrue(turretOperatorLeft);

      /*TurretCommand turretRight=new TurretCommand(m_turret,-Constants.TurretConstants.turretSpeed);
      POVButton turretRightButton=new POVButton(m_xBoxDriver,Constants.ButtonBindings.driverturretPOVRight);
      turretRightButton.whileTrue(turretRight);*/

      TurretPosCommand turret90Driv=new TurretPosCommand(m_turret,Constants.TurretConstants.turret90Pos,Constants.TurretConstants.turretMoveTimeOut,Constants.TurretConstants.deadband);

      AmpScoreHopeCommand noteFlipDriv=new AmpScoreHopeCommand(m_shooter,m_intake,m_flip,Constants.ShooterConstants.shooterAmpScoreAngle);
      POVButton noteFlipButtonDriv=new POVButton(m_xBoxDriver,ButtonBindings.driverPOVDown);
      SequentialCommandGroup ampSequentialDriv=new SequentialCommandGroup(turret90Driv,noteFlipDriv);
      noteFlipButtonDriv.onTrue(ampSequentialDriv);

      
      TurretCommand turretOperatorRight=new TurretCommand(m_turret,-Constants.TurretConstants.turretSpeed);
      POVButton turretOperatorRightButton=new POVButton(m_xBoxOperator,Constants.ButtonBindings.operatorturretPOVRight);
      turretOperatorRightButton.whileTrue(turretOperatorRight);

      ShootCommand shoot1=new ShootCommand(m_shooter,m_intake,Constants.ShooterConstants.testShooterSpeed1,Constants.ShooterConstants.baseMotorSpeed,m_shouldUseDashBoardValues,true);
      JoystickButton shootButton1=new JoystickButton(m_xBoxDriver,Constants.ButtonBindings.drivermanualShootButton);
      shootButton1.toggleOnTrue(shoot1);

      SmartShooter smartToggleTurret=new SmartShooter(m_shooter, m_turret, m_swerve, m_TurretLimelight, m_intake, false,false,true,true);
      Trigger smartToggleTurretButton= new Trigger(() -> m_xBoxOperator.getRawAxis(leftTriggerAxis)>Constants.ButtonBindings.triggerDeadband);
      smartToggleTurretButton.toggleOnTrue(smartToggleTurret);

      SmartShooter smartShooter=new SmartShooter(m_shooter, m_turret, m_swerve, m_TurretLimelight, m_intake, false,false,true,false);
      Trigger shootOpTrigger= new Trigger(() -> m_xBoxOperator.getRawAxis(rightTriggerAxis)>Constants.ButtonBindings.triggerDeadband);
      shootOpTrigger.onTrue(smartShooter);

      //ShootRotateSetReferenceCommand setref=new ShootRotateSetReferenceCommand(m_shooter,Constants.ShooterConstants.shortShotEncoderVal);
      //JoystickButton setrefButton= new JoystickButton(m_xBoxOperator,Constants.ButtonBindings.operatorShooterIntake);
      //setrefButton.whileTrue(setref);
      //Trigger setrefTrigger= new Trigger(() -> m_xBoxOperator.getRawAxis(rightTriggerAxis)>Constants.ButtonBindings.triggerDeadband);
      //setrefTrigger.whileTrue(setref);

      TurretPosCommand turret90=new TurretPosCommand(m_turret,Constants.TurretConstants.turret90Pos,Constants.TurretConstants.turretMoveTimeOut,Constants.TurretConstants.deadband);

      AmpScoreHopeCommand noteFlip=new AmpScoreHopeCommand(m_shooter,m_intake,m_flip,Constants.ShooterConstants.shooterAmpScoreAngle);
      JoystickButton noteFlipButton=new JoystickButton(m_xBoxOperator,ButtonBindings.operatorNoteFlip);
      SequentialCommandGroup ampSequential=new SequentialCommandGroup(turret90,noteFlip);
      noteFlipButton.onTrue(ampSequential);

      ShootCommand shooterIntakeOp=new ShootCommand(m_shooter,m_intake,Constants.ShooterConstants.shooterIntakeSpeed,Constants.ShooterConstants.baseMotorSpeed,m_shouldUseDashBoardValues,true);
      JoystickButton shootIntakeOpButton= new JoystickButton(m_xBoxDriver,2);
      shootIntakeOpButton.whileTrue(shooterIntakeOp);

      //FeedRollerCommand feedRoller= new FeedRollerCommand(m_shooter,Constants.ShooterConstants.manualfeederSpeed,
      //                              m_xBoxDriver);
      //JoystickButton feedRollerButton = new JoystickButton(m_xBoxDriver,ButtonBindings.drivermanualFeedRollerButton);
      //feedRollerButton.whileTrue(feedRoller);

      GyroResetCommand gyroReset= new GyroResetCommand(m_swerve);
      JoystickButton gyroResetButton = new JoystickButton(m_xBoxDriver,ButtonBindings.driverGyroResetButton);
      gyroResetButton.onTrue(gyroReset);


      TurretToggleCommand toggleTurretOp=new TurretToggleCommand(m_turret, TurretConstants.turretMoveTimeOut);
      JoystickButton turretToggleButtonOp=new JoystickButton(m_xBoxOperator,Constants.ButtonBindings.operatorTurretToggle);
      turretToggleButtonOp.onTrue(toggleTurretOp);

      TurretToggleCommand toggleTurretDriv=new TurretToggleCommand(m_turret, TurretConstants.turretMoveTimeOut);
      JoystickButton turretToggleButtonDriv=new JoystickButton(m_xBoxDriver,Constants.ButtonBindings.driverTurretToggle);
      turretToggleButtonDriv.onTrue(toggleTurretDriv);
    
      ShootRotateCommand shooterRotateUp = new ShootRotateCommand(m_shooter, Constants.ShooterConstants.rotateSlowSpeed);
      POVButton rotateUp = new POVButton(m_xBoxOperator, ButtonBindings.operatorTurretPOVrotateUp);
      rotateUp.whileTrue(shooterRotateUp);

      ShootRotateCommand shooterRotateDown = new ShootRotateCommand(m_shooter, -Constants.ShooterConstants.rotateSlowSpeed);
      POVButton rotateDown = new POVButton(m_xBoxOperator, ButtonBindings.operatorTurretPOVrotateDown);
      rotateDown.whileTrue(shooterRotateDown);

      //use left Y
      ShootRotateCommand shooterRotateStick = new ShootRotateCommand(m_shooter, ()->-m_xBoxOperator.getRawAxis(Constants.ButtonBindings.operatorRotateAxis));
      //is true when stick is outside of the deadband

      TurretCommand turretRotateStick = new TurretCommand(m_turret,()->m_xBoxOperator.getRawAxis(Constants.ButtonBindings.operatorTurretAxis));

      //ClimbCommand climbRotateStick = new ClimbCommand(m_climb,()->m_xBoxOperator.getRawAxis(Constants.ButtonBindings.operatorClimbAxis));
     
      Trigger rotateStickTrigger= new Trigger(() -> Math.abs(m_xBoxOperator.getRawAxis(Constants.ButtonBindings.operatorRotateAxis))>
      Constants.ButtonBindings.operatorRotateDeadband);
      rotateStickTrigger.whileTrue(shooterRotateStick);

      Trigger turretStickTrigger= new Trigger(() -> Math.abs(m_xBoxOperator.getRawAxis(Constants.ButtonBindings.operatorTurretAxis))>
      Constants.ButtonBindings.operatorRotateDeadband);
      turretStickTrigger.whileTrue(turretRotateStick);

      //Trigger climbStickTrigger= new Trigger(() -> Math.abs(m_xBoxOperator.getRawAxis(Constants.ButtonBindings.operatorClimbAxis))>
      //Constants.ButtonBindings.operatorRotateDeadband);
      //climbStickTrigger.whileTrue(climbRotateStick);

      ShootRotateSetReferenceCommand stowCommand = new ShootRotateSetReferenceCommand(m_shooter,Constants.ShooterConstants.shooterStowAngle);
      JoystickButton operatorStow=new JoystickButton(m_xBoxOperator,Constants.ButtonBindings.operatorStow);
      SequentialCommandGroup stowAndStop=new SequentialCommandGroup(new HardStopShooter(m_shooter),stowCommand);
      operatorStow.onTrue(stowAndStop);
   
      ShootRotateSetReferenceCommand stowDCommand = new ShootRotateSetReferenceCommand(m_shooter,Constants.ShooterConstants.shooterStowAngle);
      JoystickButton driverStow=new JoystickButton(m_xBoxDriver,Constants.ButtonBindings.driverStow);
       SequentialCommandGroup stowAndStopD=new SequentialCommandGroup(new HardStopShooter(m_shooter),stowDCommand);
       driverStow.onTrue(stowAndStopD);

      HomeShooterCommand reHomeCustom=new HomeShooterCommand(m_shooter);
      JoystickButton reHomeCustomButton = new JoystickButton(m_CustomController,3);
      reHomeCustomButton.onTrue(reHomeCustom);

      ShootCommand shootCustom1=new ShootCommand(m_shooter,m_intake,Constants.customShot1Velocity1,Constants.customShot1Velocity2,Constants.customShot1Angle,true,false);
      JoystickButton custom1 = new JoystickButton(m_CustomController, Constants.ButtonBindings.customShot1);
      custom1.onTrue(shootCustom1);

      ShootCommand shootCustom2=new ShootCommand(m_shooter,m_intake,Constants.customShot2Velocity1,Constants.customShot2Velocity2,Constants.customShot2Angle,true,false);
      JoystickButton custom2 = new JoystickButton(m_CustomController, Constants.ButtonBindings.customShot2);
      custom2.onTrue(shootCustom2);

      ShootCommand shootCustom3=new ShootCommand(m_shooter,m_intake,Constants.customShot3Velocity1,Constants.customShot3Velocity2,Constants.customShot3Angle,true,false);
      JoystickButton custom3 = new JoystickButton(m_CustomController, Constants.ButtonBindings.customShot3);
      custom3.onTrue(shootCustom3);


      ShooterNotePass notePassCustom1Prime=new ShooterNotePass(m_shooter,m_turret,m_intake,Constants.notePass10Velocity1,Constants.notePass10Velocity2,Constants.notePass10Angle,true);
      JoystickButton custom4 = new JoystickButton(m_CustomController, Constants.ButtonBindings.customShot4);
      custom4.onTrue(notePassCustom1Prime);

      ShooterNotePass notePassCustom2Prime=new ShooterNotePass(m_shooter,m_turret,m_intake,Constants.notePass11Velocity1,Constants.notePass11Velocity2,Constants.notePass11Angle,true);
      JoystickButton custom5 = new JoystickButton(m_CustomController, Constants.ButtonBindings.customShot5);
      custom5.onTrue(notePassCustom2Prime);

      /*ShooterNotePass notePassCustom3Prime=new ShooterNotePass(m_shooter,m_turret,m_intake,Constants.notePass12Velocity1,Constants.notePass12Velocity1,Constants.notePass12Angle,true);
      JoystickButton custom6 = new JoystickButton(m_CustomController, Constants.ButtonBindings.customShot6);
      custom6.onTrue(notePassCustom3Prime);*/

      ShooterNotePass notePassCustom1=new ShooterNotePass(m_shooter,m_turret,m_intake,Constants.notePass10Velocity1,Constants.notePass10Velocity2,Constants.notePass10Angle,false);
      JoystickButton custom7 = new JoystickButton(m_CustomController, Constants.ButtonBindings.customShot7);
      custom7.whileTrue(notePassCustom1);

      ShooterNotePass notePassCustom2=new ShooterNotePass(m_shooter,m_turret,m_intake,Constants.notePass11Velocity1,Constants.notePass11Velocity2,Constants.notePass11Angle,false);
      JoystickButton custom8 = new JoystickButton(m_CustomController, Constants.ButtonBindings.customShot8);
      custom8.whileTrue(notePassCustom2);

      /*ShooterNotePass notePassCustom3=new ShooterNotePass(m_shooter,m_turret,m_intake,Constants.notePass12Velocity1,Constants.notePass12Velocity1,Constants.notePass12Angle,false);
      JoystickButton custom9 = new JoystickButton(m_CustomController, Constants.ButtonBindings.customShot9);
      custom9.whileTrue(notePassCustom3);*/

      //ShootCommand shootCustom4=new ShootCommand(m_shooter,m_intake,Constants.customShot4Velocity1,Constants.customShot4Velocity2,Constants.customShot4Angle,false);
      //SourceIntakeCommand intakeCustom4=new SourceIntakeCommand(m_shooter,ShooterConstants.shooterSourceIntakeAngle);
      //JoystickButton intakeCustom4Button = new JoystickButton(m_CustomController, Constants.ButtonBindings.customShot4);
      //intakeCustom4Button.whileTrue(intakeCustom4);

      //ShootRotateSetReferenceCommand test=new ShootRotateSetReferenceCommand(m_shooter,Constants.ShooterConstants.PIDTESTAngle);
      //JoystickButton testButton=new JoystickButton(m_xBoxDriver,ButtonBindings.driverturret90);
      //testButton.whileTrue(test);

       ShootCommand shootCustom1Driv=new ShootCommand(m_shooter,m_intake,Constants.customShot1Velocity1,Constants.customShot1Velocity2,Constants.customShot1Angle,true,false);
      POVButton custom1Driv = new POVButton(m_xBoxDriver, Constants.ButtonBindings.driverPOVLeft);
      custom1Driv.onTrue(shootCustom1Driv);

      ShootCommand shootCustom2Driv=new ShootCommand(m_shooter,m_intake,-30,-30,0,true,false);
      POVButton custom2Driv = new POVButton(m_xBoxDriver, Constants.ButtonBindings.driverPOVUp);
      custom2Driv.onTrue(shootCustom2Driv);

      ShootCommand shootCustom3Driv=new ShootCommand(m_shooter,m_intake,Constants.customShot3Velocity1,Constants.customShot3Velocity2,Constants.customShot3Angle,true,false);
      POVButton custom3Driv = new POVButton(m_xBoxDriver, Constants.ButtonBindings.driverPOVRight);
      custom3Driv.onTrue(shootCustom3Driv);

    }
       
    private void refreshSmartDashboard(){  
      //TODO: ADD BACK WHEN LIMELIGHT ON
      //m_TurretLimelight.LimeLightPeriodic(true);
//      m_swerve.getPose();
        System.out.println(m_TurretLimelight.getDistance());
      //if(m_turret.IsAtLeftLimit()||m_turret.IsAtRightLimit()){
      //  m_blinkin.SetLEDPrimaryState(LEDStates.ISATLIMIT);
      //}else{
      
    }
    
    

   /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */

  private void createAutonomousCommandList(){
      try{
        SmartDashboard.putData("Auto Chooser",m_autoChooser);
        m_autoChooser.setDefaultOption(AutoConstants.autoMode0,AutoConstants.autoMode0);
        m_autoChooser.addOption(AutoConstants.autoMode1,AutoConstants.autoMode1);
        m_autoChooser.addOption(AutoConstants.autoMode2,AutoConstants.autoMode2);
        m_autoChooser.addOption(AutoConstants.autoMode3,AutoConstants.autoMode3);
        m_autoChooser.addOption(AutoConstants.autoMode4,AutoConstants.autoMode4);
        m_autoChooser.addOption(AutoConstants.autoMode5,AutoConstants.autoMode5);
        m_autoChooser.addOption(AutoConstants.autoMode6,AutoConstants.autoMode6);
        m_autoChooser.addOption(AutoConstants.autoMode7,AutoConstants.autoMode7);
        m_autoChooser.addOption(AutoConstants.autoMode8,AutoConstants.autoMode8);
       
        SmartDashboard.putData("Auto Chooser",m_autoChooser);
        m_autoDelay.setDefaultOption(AutoConstants.TimeDelay0,AutoConstants.TimeDelayVal0);
        m_autoDelay.addOption(AutoConstants.TimeDelay1,AutoConstants.TimeDelayVal1);
        m_autoDelay.addOption(AutoConstants.TimeDelay2,AutoConstants.TimeDelayVal2);
        m_autoDelay.addOption(AutoConstants.TimeDelay3,AutoConstants.TimeDelayVal3);
        m_autoDelay.addOption(AutoConstants.TimeDelay4,AutoConstants.TimeDelayVal4);
        SmartDashboard.putData("Auto Delay",m_autoDelay);

        m_autoStart.setDefaultOption(AutoConstants.Center,AutoConstants.StartingLocations.CENTER1);
        m_autoStart.addOption(AutoConstants.LeftAmp,AutoConstants.StartingLocations.LEFTAMP);
        m_autoStart.addOption(AutoConstants.LeftSpeaker,AutoConstants.StartingLocations.LEFTAMP);
        m_autoStart.addOption(AutoConstants.RightSpeaker,AutoConstants.StartingLocations.RIGHTSPEAKER);
        m_autoStart.addOption(AutoConstants.testStart,AutoConstants.StartingLocations.TESTSTART);
        SmartDashboard.putData("Auto Start",m_autoStart);

        m_autoPath1.setDefaultOption(AutoConstants.noPath, null);
        m_autoPath1.addOption(AutoConstants.blueShortAmpNote,AutoConstants.blueShortAmpAutoPoses);
        m_autoPath1.addOption(AutoConstants.blueShortCenterNote,AutoConstants.blueShortCenterAutoPoses);
        m_autoPath1.addOption(AutoConstants.blueShortSourceNote,AutoConstants.blueShortSourceAutoPoses);
        m_autoPath1.addOption(AutoConstants.redShortAmpNote,AutoConstants.redShortAmpAutoPoses);
        m_autoPath1.addOption(AutoConstants.redShortCenterNote,AutoConstants.redShortCenterAutoPoses);
        m_autoPath1.addOption(AutoConstants.redShortSourceNote,AutoConstants.redShortSourceAutoPoses);
        //m_autoPath1.addOption(AutoConstants.redSourceMoveOut,AutoConstants.moveOutRedSourceAutoPose);
        //m_autoPath1.addOption(AutoConstants.redAmpMoveOut,AutoConstants.moveOutRedAmpAutoPose);
        //m_autoPath1.addOption(AutoConstants.blueSourceMoveOut,AutoConstants.moveOutBlueSourceAutoPose);
        //m_autoPath1.addOption(AutoConstants.blueAmpMoveOut,AutoConstants.moveOutBlueSourceAutoPose);
        m_autoPath1.addOption(AutoConstants.longAmpNote1,AutoConstants.longAmpAutoPoses);
        m_autoPath1.addOption(AutoConstants.longAmpNote2,AutoConstants.longAmp2AutoPoses);
        m_autoPath1.addOption(AutoConstants.longCenterNote,AutoConstants.longCenterAutoPoses);
        m_autoPath1.addOption(AutoConstants.longSourceNote1,AutoConstants.longSourceAutoPoses);
        m_autoPath1.addOption(AutoConstants.longSourceNote2,AutoConstants.longSource2AutoPoses);
        SmartDashboard.putData("Auto Path 1",m_autoPath1);

        m_autoPath2.setDefaultOption(AutoConstants.noPath, null);
        m_autoPath2.addOption(AutoConstants.blueShortAmpNote,AutoConstants.blueShortAmpAutoPoses);
        m_autoPath2.addOption(AutoConstants.blueShortCenterNote,AutoConstants.blueShortCenterAutoPoses);
        m_autoPath2.addOption(AutoConstants.blueShortSourceNote,AutoConstants.blueShortSourceAutoPoses);
        m_autoPath2.addOption(AutoConstants.redShortAmpNote,AutoConstants.redShortAmpAutoPoses);
        m_autoPath2.addOption(AutoConstants.redShortCenterNote,AutoConstants.redShortCenterAutoPoses);
        m_autoPath2.addOption(AutoConstants.redShortSourceNote,AutoConstants.redShortSourceAutoPoses);
        m_autoPath2.addOption(AutoConstants.longAmpNote1,AutoConstants.longAmpAutoPoses);
        m_autoPath2.addOption(AutoConstants.longAmpNote2,AutoConstants.longAmp2AutoPoses);
        m_autoPath2.addOption(AutoConstants.longCenterNote,AutoConstants.longCenterAutoPoses);
        m_autoPath2.addOption(AutoConstants.longSourceNote1,AutoConstants.longSourceAutoPoses);
        m_autoPath2.addOption(AutoConstants.longSourceNote2,AutoConstants.longSource2AutoPoses);
        SmartDashboard.putData("Auto Path 2",m_autoPath2);

        m_autoPath3.setDefaultOption(AutoConstants.noPath, null);
        m_autoPath3.addOption(AutoConstants.blueShortAmpNote,AutoConstants.blueShortAmpAutoPoses);
        m_autoPath3.addOption(AutoConstants.blueShortCenterNote,AutoConstants.blueShortCenterAutoPoses);
        m_autoPath3.addOption(AutoConstants.blueShortSourceNote,AutoConstants.blueShortSourceAutoPoses);
        m_autoPath3.addOption(AutoConstants.redShortAmpNote,AutoConstants.redShortAmpAutoPoses);
        m_autoPath3.addOption(AutoConstants.redShortCenterNote,AutoConstants.redShortCenterAutoPoses);
        m_autoPath3.addOption(AutoConstants.redShortSourceNote,AutoConstants.redShortSourceAutoPoses);
        m_autoPath3.addOption(AutoConstants.longAmpNote1,AutoConstants.longAmpAutoPoses);
        m_autoPath3.addOption(AutoConstants.longAmpNote2,AutoConstants.longAmp2AutoPoses);
        m_autoPath3.addOption(AutoConstants.longCenterNote,AutoConstants.longCenterAutoPoses);
        m_autoPath3.addOption(AutoConstants.longSourceNote1,AutoConstants.longSourceAutoPoses);
        m_autoPath3.addOption(AutoConstants.longSourceNote2,AutoConstants.longSource2AutoPoses);
        SmartDashboard.putData("Auto Path 3",m_autoPath3);

        m_autoPath4.setDefaultOption(AutoConstants.noPath, null);
        m_autoPath4.addOption(AutoConstants.blueShortAmpNote,AutoConstants.blueShortAmpAutoPoses);
        m_autoPath4.addOption(AutoConstants.blueShortCenterNote,AutoConstants.blueShortCenterAutoPoses);
        m_autoPath4.addOption(AutoConstants.blueShortSourceNote,AutoConstants.blueShortSourceAutoPoses);
        m_autoPath4.addOption(AutoConstants.redShortAmpNote,AutoConstants.redShortAmpAutoPoses);
        m_autoPath4.addOption(AutoConstants.redShortCenterNote,AutoConstants.redShortCenterAutoPoses);
        m_autoPath4.addOption(AutoConstants.redShortSourceNote,AutoConstants.redShortSourceAutoPoses);
        m_autoPath4.addOption(AutoConstants.longAmpNote1,AutoConstants.longAmpAutoPoses);
        m_autoPath4.addOption(AutoConstants.longAmpNote2,AutoConstants.longAmp2AutoPoses);
        m_autoPath4.addOption(AutoConstants.longCenterNote,AutoConstants.longCenterAutoPoses);
        m_autoPath4.addOption(AutoConstants.longSourceNote1,AutoConstants.longSourceAutoPoses);
        m_autoPath4.addOption(AutoConstants.longSourceNote2,AutoConstants.longSource2AutoPoses);
        SmartDashboard.putData("Auto Path 4",m_autoPath4);

        m_autoPath5.setDefaultOption(AutoConstants.noPath, null);
        m_autoPath5.addOption(AutoConstants.blueShortAmpNote,AutoConstants.blueShortAmpAutoPoses);
        m_autoPath5.addOption(AutoConstants.blueShortCenterNote,AutoConstants.blueShortCenterAutoPoses);
        m_autoPath5.addOption(AutoConstants.blueShortSourceNote,AutoConstants.blueShortSourceAutoPoses);
        m_autoPath5.addOption(AutoConstants.redShortAmpNote,AutoConstants.redShortAmpAutoPoses);
        m_autoPath5.addOption(AutoConstants.redShortCenterNote,AutoConstants.redShortCenterAutoPoses);
        m_autoPath5.addOption(AutoConstants.redShortSourceNote,AutoConstants.redShortSourceAutoPoses);
        m_autoPath5.addOption(AutoConstants.longAmpNote1,AutoConstants.longAmpAutoPoses);
        m_autoPath5.addOption(AutoConstants.longAmpNote2,AutoConstants.longAmp2AutoPoses);
        m_autoPath5.addOption(AutoConstants.longCenterNote,AutoConstants.longCenterAutoPoses);
        m_autoPath5.addOption(AutoConstants.longSourceNote1,AutoConstants.longSourceAutoPoses);
        m_autoPath5.addOption(AutoConstants.longSourceNote2,AutoConstants.longSource2AutoPoses);
        SmartDashboard.putData("Auto Path 5",m_autoPath5);

        m_shootFinalNote.setDefaultOption("Dont Shoot Final Note",false);
        m_shootFinalNote.addOption("Shoot Final Note",true);
        SmartDashboard.putData("Should Shoot Final Note",m_shootFinalNote);

        m_shouldUseDashBoardValues.setDefaultOption("Dont Use Shooter Vals",false);
        m_shouldUseDashBoardValues.addOption("Use Shooter Vals",true);
        SmartDashboard.putData("Should Use Inputted Shooter Vals",m_shouldUseDashBoardValues);

        SmartDashboard.putNumber("Shooter1Veloc",0);
        SmartDashboard.putNumber("Shooter2Veloc",0);
        SmartDashboard.putNumber("ShooterAngle",0);
      } catch (Exception e){
          System.out.println("Create Auto Exception: " + e.getMessage());
          }
      }

  public Command getAutonomousCommand(){
    Command returnCommand;
    String autoChosen=m_autoChooser.getSelected();
  
    //check to see if using Choose Your Own Adventure or Defined Path
    if (autoChosen==Constants.AutoConstants.autoMode0){
      returnCommand=getChoseAdventureAutonomousCommand();
    } else {
      returnCommand=getDefinedAutoCommand(autoChosen);
    }
    return returnCommand;

  }

  public Command getChoseAdventureAutonomousCommand(){
    Alliance currentAlliance=DriverStation.getAlliance().get();
    AutoCommands autoMaker = new AutoCommands(m_swerve,m_shooter,m_intake,m_turret,m_TurretLimelight);
    ChooseYourOwnAdventureAuto autoModularMaker = new ChooseYourOwnAdventureAuto(m_swerve,m_shooter,m_intake,m_turret,m_TurretLimelight);
    AutoConstants.StartingLocations startLocation=m_autoStart.getSelected();
    double delay=m_autoDelay.getSelected();
    String autoChosen=m_autoChooser.getSelected();
    Command newCommand=null;
    
   // if(m_TurretLimelight.TargetAvailible()){
   //   m_swerve.resetOdometry(m_TurretLimelight.GetPoseViaApriltag());
   // }else{
      m_swerve.resetOdometry(autoMaker.getStartingPose(startLocation,currentAlliance));
    //}

    newCommand=autoModularMaker.CreateAutoCommand(m_autoPath1.getSelected(),m_autoPath2.getSelected(),m_autoPath3.getSelected(),m_autoPath4.getSelected(),
                                                  m_autoPath5.getSelected());                                     
   // newCommand=createDistanceAuto();
    //newCommand=autoModularMaker.TESTAUTO(currentAlliance);

    return newCommand;
  }

  public Command getDefinedAutoCommand(String autoChosen){
    Command returnCommand;
    Alliance currentAlliance=DriverStation.getAlliance().get();
    switch(autoChosen){
      case Constants.AutoConstants.autoMode1: // Do Nothing
        returnCommand=new AutoDoNothingCommand();
        break;
      case Constants.AutoConstants.autoMode2: // 2=PP-Start Center,Score, Center Short, Amp Short
        returnCommand=PathPlanCenterScoreCenterAmp();
        break;
      case Constants.AutoConstants.autoMode3: // 5=PP-Start Center, Center Note, Amp Long
        returnCommand=PathPlanCenterScoreCenterAmpLongAmp();
        break;
      case Constants.AutoConstants.autoMode4: // 3=PP-Start Left, Amp Short, Amp Long
        returnCommand=PathPlanLeftScoreAmpLongAmp1();
        break;
      case Constants.AutoConstants.autoMode5: // 5=PP-Start Right, LongSource1, LongSource2
        returnCommand= PathPlanRightScoreLongSource1LongSource2();
        break;
      case Constants.AutoConstants.autoMode6: // 6=PP-Start Center, Center Note, Amp Note, Amp Long
        returnCommand= PathPlanAnyTwoNote();
        break;
      case Constants.AutoConstants.autoMode7: 
        returnCommand=PathPlanShootWaitMove();
        break;
      case Constants.AutoConstants.autoMode8: 
        returnCommand=legacyAutoShootMoveBackShoot(currentAlliance);
        break;       
      default:
        returnCommand=new AutoDoNothingCommand();
    }
    return returnCommand;
  }

  public void checkBrakeButton(){
    if(m_brakeButton.get() && m_brakeButtonPressed==false){ 
      m_brakeButtonPressed=true;
      if(m_isBrakeButtonToggled==false){
   //                 disableBrakeMode();
                    m_isBrakeButtonToggled=true;
            }else{
    //                resetBrakeModetoNormal();
                    m_isBrakeButtonToggled=false;
            }
  }else{

    if(m_brakeButton.get()==false){
            m_brakeButtonPressed=false;
    }
  }
  }

  public void SetBaseShooterSpeed(){
    m_shooter.SlowShootingMotors();
  }

  public void stopRumble(){
    m_xBoxDriver.setRumble(RumbleType.kBothRumble,Constants.InputControllers.kRumbleoff);
    m_xBoxOperator.setRumble(RumbleType.kBothRumble,Constants.InputControllers.kRumbleoff);
  }

  public void AutonMode(){
  // m_ledStrip.setRobotMode(LEDSChargedup.LEDMode.AUTOMODE);
  // enableLimelights();
    resetBrakeModetoNormal();
    homeRobot(); 
    ChangeVisionPipeline(); 
    m_intake.ResetToggleBoolean();
  } 
  
  public void AllPeriodic(){
       refreshSmartDashboard();
        m_TurretLimelight.LimeLightPeriodic(true); 

   
  }

  public void TeleopPeriodic(){
    boolean isTurretStraight=(m_turret.GetEncoderValue()>=-0.25||m_turret.GetEncoderValue()<=0.25);

    if(!m_shooter.isAtStow()){
      if (m_hasNoteLoopCounter>0 && m_hasNoteLoopCounter>khasNoteLoopLEDLimit){
        //reset has note
         m_candle.SetCANdleLEDS(LEDConstants.CANDLELEDStates.HASNOTE);
         m_hasNoteLoopCounter++;
      } else{
      if(!m_intake.isBeamBroken() ){
        m_candle.SetCANdleLEDS(LEDConstants.CANDLELEDStates.ROBOTFRONTSIDE);
      }else if(m_intake.isRunning()){
        m_candle.SetCANdleLEDS(LEDConstants.CANDLELEDStates.INTAKERUNNING);
      }else{
           m_candle.SetCANdleLEDS(LEDConstants.CANDLELEDStates.HASNOTE);
          m_hasNoteLoopCounter=1;
      }
    }
       m_candle.SetCANdleLEDS(LEDConstants.CANDLELEDStates.AMPSCORESIDE);
      
      if(m_TurretLimelight.isTargetAvailible()){
        if(m_turret.IsOnTarget()){
          m_candle.SetCANdleLEDS(LEDConstants.CANDLELEDStates.ISONTARGET);
        }else{
          m_candle.SetCANdleLEDS(LEDConstants.CANDLELEDStates.TARGETLOCK);
        }
      }else{
        m_candle.SetCANdleLEDS(LEDConstants.CANDLELEDStates.NOTARGET);
      }
    }else{
      m_candle.SetCANdleLEDS(LEDConstants.CANDLELEDStates.ISATSTOW);
    }  
  }

  private void setDisabledLEDS(){
        if(m_shooter.isAtHomeValue()){
          m_candle.SetCANdleLEDS(LEDConstants.CANDLELEDStates.DISABLED);
       }else{
          m_candle.SetCANdleLEDS(LEDConstants.CANDLELEDStates.DISABLEDNOHOME);
 
}
  }
  public void DisabledInit(){
    stopRumble();
    System.out.println("Putting Motors in Coast");
    disableBrakeMode(); // disable brake mode when disabled 
    m_candle.SetCANdleLEDS(LEDConstants.CANDLELEDStates.NOSTATE);
    setDisabledLEDS();
  }
   
  public void DisabledPeriodic(){
    
     setDisabledLEDS();
  }
  
  public void AutoPeriodic(){
  
    if(m_TurretLimelight.isTargetAvailible()){
    //Get Limelight
    //add Vision measurement to swerve
    //m_swerve.AddVisionPose(m_Limelight.GetPoseViaApriltag(),0,true,1);
    }
   
  }
  private void resetDefaultCommand(){
      m_swerve.setDefaultCommand(
        m_swerve.drive(
            () -> m_xBoxDriver.getRawAxis(translationAxis),
            () -> m_xBoxDriver.getRawAxis(strafeAxis),
            () -> m_xBoxDriver.getRawAxis(rotationAxis),
            () -> m_xBoxDriver.getRawAxis(rightTriggerAxis)));

  }
  public void TeleopMode(){
    //m_blinkin.SetLEDPrimaryState(BlinkinConstants.LEDStates.TELEOP);
    m_swerve.resetModules();
    resetDefaultCommand();
    resetBrakeModetoNormal();
    m_shooter.ResetControlType();  
    m_shooter.stopRotate(); //reset rogue pid
   
    m_shooter.ResetShotsTaken();
    //m_shooter.setupMusic();
    //m_shooter.playMusic(); // Test
    homeRobot();
    ChangeVisionPipeline();
    m_intake.ResetToggleBoolean();
  }

  private void ChangeVisionPipeline(){
    Alliance currentAlliance=DriverStation.getAlliance().get();

    if(currentAlliance==Alliance.Red){
      m_TurretLimelight.setPipeline(Constants.LimeLightValues.redSpeakerPipeline);
    }else{
      m_TurretLimelight.setPipeline(Constants.LimeLightValues.blueSpeakerPipeline);
    }
  }

  private void resetBrakeModetoNormal(){
    m_intake.setBrakeOn();
 //   m_shooter.setBrakeOn();
    m_turret.setBrakeOn();
 //   m_climb.setBrakeOn();
  }

  public void disableBrakeMode(){
    m_intake.setCoastOn();
    m_shooter.setCoastOn();
    m_turret.setCoastOn();
//    m_climb.setCoastOn();
  }

  private void homeRobot(){
    if(m_HasHomed==false){
      Command turretHome=new TurretHomeCommand(m_turret);
      CommandScheduler.getInstance().schedule(turretHome);

      Command shooterHome=new HomeShooterCommand(m_shooter);
      CommandScheduler.getInstance().schedule(shooterHome);
      m_HasHomed=true;
    }
  }
  private Command legacyAutoShootMoveBackShoot(){
    SmartShooter shoot0=new SmartShooter(m_shooter, m_turret, m_swerve, m_TurretLimelight, m_intake, false,false,true,false); 
    SmartShooter shoot1=new SmartShooter(m_shooter, m_turret, m_swerve, m_TurretLimelight, m_intake, false,false,true,false); 
    ShootRotateSetReferenceCommand shooterUp = new ShootRotateSetReferenceCommand(m_shooter,0);
    TurretPosCommand turretSet0=new TurretPosCommand(m_turret,Constants.TurretConstants.turretStraightPos,Constants.TurretConstants.turretMoveTimeOut,Constants.TurretConstants.deadband);
    IntakeToggleCommand startIntake1=new IntakeToggleCommand(m_intake,Constants.IntakeConstants.intakeSpeed,true);
    AutoMoveCommand moveback = new AutoMoveCommand(m_swerve,0.0, Constants.AutoConstants.distancetoShortNote);
    return new SequentialCommandGroup(shoot0,turretSet0,startIntake1,shooterUp,moveback,shoot1);
  }
    private Command legacyAutoShootMoveBackShoot(Alliance currentAlliance){
    SmartShooter shoot0=new SmartShooter(m_shooter, m_turret, m_swerve, m_TurretLimelight, m_intake, false,false,true,false); 
    SmartShooter shoot1=new SmartShooter(m_shooter, m_turret, m_swerve, m_TurretLimelight, m_intake, false,false,true,false); 
    SmartShooter shoot2=new SmartShooter(m_shooter, m_turret, m_swerve, m_TurretLimelight, m_intake, false,false,true,false); 
    ShootRotateSetReferenceCommand shooterUp0 = new ShootRotateSetReferenceCommand(m_shooter,0);
    ShootRotateSetReferenceCommand shooterUp1 = new ShootRotateSetReferenceCommand(m_shooter,0);
    
    TurretPosCommand turretSet00=new TurretPosCommand(m_turret,Constants.TurretConstants.turretStraightPos,Constants.TurretConstants.turretMoveTimeOut,Constants.TurretConstants.deadband);
    TurretPosCommand turretSet01=new TurretPosCommand(m_turret,Constants.TurretConstants.turretStraightPos,Constants.TurretConstants.turretMoveTimeOut,Constants.TurretConstants.deadband);
    TurretPosCommand turretSet90=new TurretPosCommand(m_turret,Constants.TurretConstants.turret90Pos,Constants.TurretConstants.turretMoveTimeOut,Constants.TurretConstants.deadband);
    IntakeToggleCommand startIntake1=new IntakeToggleCommand(m_intake,Constants.IntakeConstants.intakeSpeed,true);
    IntakeToggleCommand startIntake2=new IntakeToggleCommand(m_intake,Constants.IntakeConstants.intakeSpeed,true);
    AutoMoveCommand moveback = new AutoMoveCommand(m_swerve,0.0, Constants.AutoConstants.distancetoShortNote);
    AutoMoveCommand moveforward = new AutoMoveCommand(m_swerve,180, Constants.AutoConstants.distancBacktoNoteLine);
    AutoMoveCommand moveover;
    AutoMoveCommand moveover2;
    if(currentAlliance==Alliance.Red){ 
      moveover = new AutoMoveCommand(m_swerve,90.0, Constants.AutoConstants.distanceBetweeNotes);
      moveover2 = new AutoMoveCommand(m_swerve,270.0, Constants.AutoConstants.distanceBetweeNotes);
    }else { // switch direction for Blue
      moveover = new AutoMoveCommand(m_swerve,90.0, Constants.AutoConstants.distanceBetweeNotes);
      moveover2 = new AutoMoveCommand(m_swerve,270.0, Constants.AutoConstants.distanceBetweeNotes);

    }
      return new SequentialCommandGroup(shoot0,turretSet00,startIntake1,shooterUp0,moveback,shoot1); 
                                        //,moveforward,turretSet90,
                                        //shooterUp1,startIntake2,moveover,turretSet01,moveover2,shoot2);
  }

  public Command PathPlanCenterScoreCenterAmp(){
    ChooseYourOwnAdventureAuto autoBuilder=new ChooseYourOwnAdventureAuto(m_swerve, m_shooter, m_intake, m_turret, m_TurretLimelight);
    Alliance currentAlliance=DriverStation.getAlliance().get();
    Pose2d centerStartPos=autoBuilder.getStartingPose(AutoConstants.StartingLocations.CENTER1,currentAlliance);
    Pose2d centerNotePos=autoBuilder.getNotePose(AutoConstants.NotePoses.CENTERNOTEPOS,currentAlliance);
    Pose2d ampNotePos=autoBuilder.getNotePose(AutoConstants.NotePoses.AMPNOTEPOS,currentAlliance);
    Pose2d ampWaypoint=autoBuilder.getNotePose(AutoConstants.NotePoses.AMPNOTEWAYPOINT, currentAlliance);

    SmartShooter shoot1=new SmartShooter(m_shooter, m_turret, m_swerve, m_TurretLimelight, m_intake,false,false,false,false);
    SmartShooter shoot2=new SmartShooter(m_shooter, m_turret, m_swerve, m_TurretLimelight, m_intake,false,false,false,false);
    SmartShooter shoot3=new SmartShooter(m_shooter, m_turret, m_swerve, m_TurretLimelight, m_intake,false,false,false,false);

    TurretPosCommand turretSet00=new TurretPosCommand(m_turret,Constants.TurretConstants.turretStraightPos,Constants.TurretConstants.turretMoveTimeOut,Constants.TurretConstants.deadband);
    TurretPosCommand turretSet01=new TurretPosCommand(m_turret,Constants.TurretConstants.turretStraightPos,Constants.TurretConstants.turretMoveTimeOut,Constants.TurretConstants.deadband);
    TurretPosCommand turretSet90=new TurretPosCommand(m_turret,Constants.TurretConstants.turret90Pos,Constants.TurretConstants.turretMoveTimeOut,Constants.TurretConstants.deadband);

    IntakeToggleCommand intake1=new IntakeToggleCommand(m_intake,IntakeConstants.autoIntakeSpeed,true);
    IntakeToggleCommand intake2=new IntakeToggleCommand(m_intake,IntakeConstants.autoIntakeSpeed,true);
  
    ShootRotateSetReferenceCommand shooterSetRef1=new ShootRotateSetReferenceCommand(m_shooter,0);
    ShootRotateSetReferenceCommand shooterSetRef2=new ShootRotateSetReferenceCommand(m_shooter,0);

    Command startToCenterNote=m_swerve.createPathCommand(autoBuilder.CreateAutoPath(centerStartPos,centerNotePos));
    Command centerNoteToAmpNote=m_swerve.createPathCommand(autoBuilder.CreateAutoPath(centerNotePos,ampNotePos));
    Command centerNoteToAmpWaypoint=m_swerve.createPathCommand(autoBuilder.CreateAutoPath(centerNotePos,ampWaypoint));
    Command ampwaypointToAmpNote=m_swerve.createPathCommand(autoBuilder.CreateAutoPath(ampWaypoint,ampNotePos));
    Command ampNoteToCenterNote=m_swerve.createPathCommand(autoBuilder.CreateAutoPath(ampNotePos,centerNotePos));

    ParallelCommandGroup newMoveSetRef1=new ParallelCommandGroup(turretSet00,startToCenterNote,shooterSetRef1);
    ParallelCommandGroup newMoveSetRef2=new ParallelCommandGroup(turretSet90,centerNoteToAmpWaypoint,shooterSetRef2);
    SequentialCommandGroup ampWaypointSeq=new SequentialCommandGroup(newMoveSetRef2,ampwaypointToAmpNote);
    ParallelCommandGroup newMoveSetRef3=new ParallelCommandGroup(turretSet01,ampNoteToCenterNote);

    m_swerve.resetOdometry(centerStartPos);
    SequentialCommandGroup newSeq=new SequentialCommandGroup(shoot1,intake1,newMoveSetRef1,shoot2,intake2,ampWaypointSeq,newMoveSetRef3,shoot3);

    return newSeq;
  }

  public Command PathPlanLeftScoreAmpLongAmp1(){
    ChooseYourOwnAdventureAuto autoBuilder=new ChooseYourOwnAdventureAuto(m_swerve, m_shooter, m_intake, m_turret, m_TurretLimelight);
    Alliance currentAlliance=DriverStation.getAlliance().get();
    Pose2d leftStartPos=autoBuilder.getStartingPose(AutoConstants.StartingLocations.LEFTSPEAKER,currentAlliance);
    Pose2d shortAmpNotePos=autoBuilder.getNotePose(AutoConstants.NotePoses.AMPNOTEPOS,currentAlliance);
    Pose2d longAmp1NotePos=autoBuilder.getNotePose(AutoConstants.NotePoses.LONGAMP1POS,currentAlliance);
    Pose2d shortAmpShootPos=autoBuilder.getNotePose(AutoConstants.NotePoses.AMPSHOOTPOS, currentAlliance);
    double turretAngle=autoBuilder.getTurretAngle(currentAlliance,AutoConstants.ShootLocations.LEFTSPEAKER);
    double ampShootAngle=autoBuilder.getTurretAngle(currentAlliance,AutoConstants.ShootLocations.AMPSHOOT);
    double wingShootAngle=autoBuilder.getTurretAngle(currentAlliance,AutoConstants.ShootLocations.LEFTWINGSHOOTPOS);

    SmartShooter shoot1=new SmartShooter(m_shooter, m_turret, m_swerve, m_TurretLimelight, m_intake,false,false,false,false);
    SmartShooter shoot2=new SmartShooter(m_shooter, m_turret, m_swerve, m_TurretLimelight, m_intake,false,false,false,false);
    SmartShooter shoot3=new SmartShooter(m_shooter, m_turret, m_swerve, m_TurretLimelight, m_intake,false,false,false,false);

    TurretPosCommand turretSet00=new TurretPosCommand(m_turret,Constants.TurretConstants.turretStraightPos,Constants.TurretConstants.turretMoveTimeOut,Constants.TurretConstants.deadband);
    TurretPosCommand turretSet01=new TurretPosCommand(m_turret,Constants.TurretConstants.turretStraightPos,Constants.TurretConstants.turretMoveTimeOut,Constants.TurretConstants.deadband);
    TurretPosCommand turretSetLeftShoot1=new TurretPosCommand(m_turret,turretAngle,Constants.TurretConstants.turretMoveTimeOut,Constants.TurretConstants.deadband);
    TurretPosCommand turretSetAmpShoot=new TurretPosCommand(m_turret,ampShootAngle,Constants.TurretConstants.turretMoveTimeOut,Constants.TurretConstants.deadband);
    TurretPosCommand turretSetWingShoot=new TurretPosCommand(m_turret,wingShootAngle,Constants.TurretConstants.turretMoveTimeOut,Constants.TurretConstants.deadband);

    IntakeToggleCommand intake1=new IntakeToggleCommand(m_intake,IntakeConstants.autoIntakeSpeed,true);
    IntakeToggleCommand intake2=new IntakeToggleCommand(m_intake,IntakeConstants.autoIntakeSpeed,true);
  
    ShootRotateSetReferenceCommand shooterSetRef1=new ShootRotateSetReferenceCommand(m_shooter,0);
    ShootRotateSetReferenceCommand shooterSetRef2=new ShootRotateSetReferenceCommand(m_shooter,0);

    Command startToAmpNote=m_swerve.createPathCommand(autoBuilder.CreateAutoPath(leftStartPos,shortAmpNotePos));
    Command shortAmpNoteToLongAmpNote=m_swerve.createPathCommand(autoBuilder.CreateAutoPath(shortAmpNotePos,longAmp1NotePos));
    Command longAmpNoteToAmpNotePos=m_swerve.createPathCommand(autoBuilder.CreateAutoPath(longAmp1NotePos,shortAmpShootPos));

    ParallelCommandGroup newMoveSetRef1=new ParallelCommandGroup(turretSet00,startToAmpNote,shooterSetRef1);
    ParallelCommandGroup newMoveSetRef2=new ParallelCommandGroup(turretSet01,shortAmpNoteToLongAmpNote,shooterSetRef2);
    ParallelCommandGroup newMoveSetRef3=new ParallelCommandGroup(turretSetWingShoot,longAmpNoteToAmpNotePos);

    m_swerve.resetOdometry(leftStartPos);

    SequentialCommandGroup newSeq=new SequentialCommandGroup(turretSetLeftShoot1,shoot1,intake1,newMoveSetRef1,turretSetAmpShoot,shoot2,intake2,newMoveSetRef2,newMoveSetRef3,shoot3);

    return newSeq;
  }

  public Command PathPlanShootWaitMove(){
    ChooseYourOwnAdventureAuto autoBuilder=new ChooseYourOwnAdventureAuto(m_swerve, m_shooter, m_intake, m_turret, m_TurretLimelight);
    Alliance currentAlliance=DriverStation.getAlliance().get();
    Pose2d leftStartPos=autoBuilder.getStartingPose(AutoConstants.StartingLocations.LEFTSPEAKER,currentAlliance);
    Pose2d outOfStartPos=autoBuilder.getNotePose(AutoConstants.NotePoses.OUTOFBOUNDPOS,currentAlliance);
    DelayCommand delay=new DelayCommand(AutoConstants.pathPlanDelay);

    SmartShooter shoot1=new SmartShooter(m_shooter, m_turret, m_swerve, m_TurretLimelight, m_intake,false,false,true,false);

    TurretPosCommand turretSet00=new TurretPosCommand(m_turret,Constants.TurretConstants.turretStraightPos,Constants.TurretConstants.turretMoveTimeOut,Constants.TurretConstants.deadband);
  
    ShootRotateSetReferenceCommand shooterSetRef1=new ShootRotateSetReferenceCommand(m_shooter,0);

    Command startToAmpNote=m_swerve.createPathCommand(autoBuilder.CreateAutoPath(leftStartPos,outOfStartPos));

    ParallelCommandGroup newMoveSetRef1=new ParallelCommandGroup(turretSet00,startToAmpNote,shooterSetRef1);

    m_swerve.resetOdometry(leftStartPos);

    SequentialCommandGroup newSeq=new SequentialCommandGroup(shoot1,delay,newMoveSetRef1);

    return newSeq;
  }

  public Command PathPlanAnyTwoNote(){
    ChooseYourOwnAdventureAuto autoBuilder=new ChooseYourOwnAdventureAuto(m_swerve, m_shooter, m_intake, m_turret, m_TurretLimelight);
    Alliance currentAlliance=DriverStation.getAlliance().get();
    Pose2d startPos=autoBuilder.getStartingPose(AutoConstants.StartingLocations.CENTER1,currentAlliance);
    Pose2d notePos=autoBuilder.getNotePose(AutoConstants.NotePoses.CENTERNOTEPOS,currentAlliance);
    Pose2d moveOutPos=autoBuilder.getNotePose(AutoConstants.NotePoses.TWONOTEMOVEOUT,currentAlliance);
    double startTurretAngle=autoBuilder.getTurretAngle(currentAlliance,AutoConstants.ShootLocations.RIGHTSPEAKER);

    SmartShooter shoot1=new SmartShooter(m_shooter, m_turret, m_swerve, m_TurretLimelight, m_intake,false,false,true,false);
    SmartShooter shoot2=new SmartShooter(m_shooter, m_turret, m_swerve, m_TurretLimelight, m_intake,false,false,true,false);

    TurretPosCommand turretSet00=new TurretPosCommand(m_turret,Constants.TurretConstants.turretStraightPos,Constants.TurretConstants.turretMoveTimeOut,Constants.TurretConstants.deadband);
    IntakeToggleCommand intake1=new IntakeToggleCommand(m_intake,IntakeConstants.autoIntakeSpeed,true);
  
    ShootRotateSetReferenceCommand shooterSetRef1=new ShootRotateSetReferenceCommand(m_shooter,0);

    Command startToMoveOutPos=m_swerve.createPathCommand(autoBuilder.CreateAutoPath(startPos,moveOutPos));
    Command moveOutPosToAmpNote=m_swerve.createPathCommand(autoBuilder.CreateAutoPath(moveOutPos,notePos));

    ParallelCommandGroup newMoveSetRef1=new ParallelCommandGroup(turretSet00,moveOutPosToAmpNote,shooterSetRef1);

    m_swerve.resetOdometry(startPos);

    SequentialCommandGroup newSeq=new SequentialCommandGroup(startToMoveOutPos,shoot1,intake1,newMoveSetRef1,shoot2);

    return newSeq;
  }

  public Command PathPlanRightScoreSource1(){
    ChooseYourOwnAdventureAuto autoBuilder=new ChooseYourOwnAdventureAuto(m_swerve, m_shooter, m_intake, m_turret, m_TurretLimelight);
    Alliance currentAlliance=DriverStation.getAlliance().get();
    Pose2d rightStartPos=autoBuilder.getStartingPose(AutoConstants.StartingLocations.RIGHTSPEAKER,currentAlliance);
    Pose2d longSource1NotePos=autoBuilder.getNotePose(AutoConstants.NotePoses.LONGSOURCE1POS,currentAlliance);
    double turretAngle=autoBuilder.getTurretAngle(currentAlliance,AutoConstants.ShootLocations.RIGHTSPEAKER);

    SmartShooter shoot1=new SmartShooter(m_shooter, m_turret, m_swerve, m_TurretLimelight, m_intake,false,false,true,false);
    SmartShooter shoot2=new SmartShooter(m_shooter, m_turret, m_swerve, m_TurretLimelight, m_intake,false,false,true,false);

    TurretPosCommand turretSet00=new TurretPosCommand(m_turret,Constants.TurretConstants.turretStraightPos,Constants.TurretConstants.turretMoveTimeOut,Constants.TurretConstants.deadband);
    TurretPosCommand turretSetRightShoot1=new TurretPosCommand(m_turret,turretAngle,Constants.TurretConstants.turretMoveTimeOut,Constants.TurretConstants.deadband);
    TurretPosCommand turretSetRightShoot2=new TurretPosCommand(m_turret,turretAngle,Constants.TurretConstants.turretMoveTimeOut,Constants.TurretConstants.deadband);

    IntakeToggleCommand intake1=new IntakeToggleCommand(m_intake,IntakeConstants.autoIntakeSpeed,true);
  
    ShootRotateSetReferenceCommand shooterSetRef1=new ShootRotateSetReferenceCommand(m_shooter,0);

    Command startToLongSourceNote=m_swerve.createPathCommand(autoBuilder.CreateAutoPath(rightStartPos,longSource1NotePos));
    Command longSourceNoteToStart=m_swerve.createPathCommand(autoBuilder.CreateAutoPath(longSource1NotePos,rightStartPos));

    ParallelCommandGroup newMoveSetRef1=new ParallelCommandGroup(turretSet00,startToLongSourceNote,shooterSetRef1);
    ParallelCommandGroup newMoveSetRef2=new ParallelCommandGroup(turretSetRightShoot2,longSourceNoteToStart,turretSetRightShoot2);

    m_swerve.resetOdometry(rightStartPos);

    SequentialCommandGroup newSeq=new SequentialCommandGroup(turretSetRightShoot1,shoot1,intake1,newMoveSetRef1,newMoveSetRef2,shoot2);

    return newSeq;
  }

  public Command PathPlanCenterScoreCenterLongAmp1(){
    ChooseYourOwnAdventureAuto autoBuilder=new ChooseYourOwnAdventureAuto(m_swerve, m_shooter, m_intake, m_turret, m_TurretLimelight);
    Alliance currentAlliance=DriverStation.getAlliance().get();
    Pose2d centerStartPos=autoBuilder.getStartingPose(AutoConstants.StartingLocations.CENTER1,currentAlliance);
    Pose2d longAmp1NotePos=autoBuilder.getNotePose(AutoConstants.NotePoses.LONGAMP1POS,currentAlliance);
    Pose2d centerNotePos=autoBuilder.getNotePose(AutoConstants.NotePoses.CENTERNOTEPOS,currentAlliance);

    SmartShooter shoot1=new SmartShooter(m_shooter, m_turret, m_swerve, m_TurretLimelight, m_intake,false,false,false,false);
    SmartShooter shoot2=new SmartShooter(m_shooter, m_turret, m_swerve, m_TurretLimelight, m_intake,false,false,false,false);
    SmartShooter shoot3=new SmartShooter(m_shooter, m_turret, m_swerve, m_TurretLimelight, m_intake,false,false,false,false);

    TurretPosCommand turretSet00=new TurretPosCommand(m_turret,Constants.TurretConstants.turretStraightPos,Constants.TurretConstants.turretMoveTimeOut,Constants.TurretConstants.deadband);
    TurretPosCommand turretSet01=new TurretPosCommand(m_turret,Constants.TurretConstants.turretStraightPos,Constants.TurretConstants.turretMoveTimeOut,Constants.TurretConstants.deadband);
    TurretPosCommand turretSet02=new TurretPosCommand(m_turret,Constants.TurretConstants.turretStraightPos,Constants.TurretConstants.turretMoveTimeOut,Constants.TurretConstants.deadband);

    IntakeToggleCommand intake1=new IntakeToggleCommand(m_intake,IntakeConstants.autoIntakeSpeed,true);
    IntakeToggleCommand intake2=new IntakeToggleCommand(m_intake,IntakeConstants.autoIntakeSpeed,true);
  
    ShootRotateSetReferenceCommand shooterSetRef1=new ShootRotateSetReferenceCommand(m_shooter,0);
    ShootRotateSetReferenceCommand shooterSetRef2=new ShootRotateSetReferenceCommand(m_shooter,0);

    Command startToCenter=m_swerve.createPathCommand(autoBuilder.CreateAutoPath(centerStartPos,centerNotePos));
    Command shortCenterToLongAmpNote=m_swerve.createPathCommand(autoBuilder.CreateAutoPath(centerNotePos,longAmp1NotePos));
    Command longAmpNoteToCenterNote=m_swerve.createPathCommand(autoBuilder.CreateAutoPath(longAmp1NotePos,centerNotePos));

    ParallelCommandGroup newMoveSetRef1=new ParallelCommandGroup(turretSet00,startToCenter,shooterSetRef1);
    ParallelCommandGroup newMoveSetRef2=new ParallelCommandGroup(turretSet01,shortCenterToLongAmpNote,shooterSetRef2);
    ParallelCommandGroup newMoveSetRef3=new ParallelCommandGroup(turretSet02,longAmpNoteToCenterNote);

    m_swerve.resetOdometry(centerStartPos);

    SequentialCommandGroup newSeq=new SequentialCommandGroup(shoot1,intake1,newMoveSetRef1,shoot2,intake2,newMoveSetRef2,newMoveSetRef3,shoot3);

    return newSeq;
  }

  public Command PathPlanCenterScoreCenterAmpLongAmp(){
    ChooseYourOwnAdventureAuto autoBuilder=new ChooseYourOwnAdventureAuto(m_swerve, m_shooter, m_intake, m_turret, m_TurretLimelight);
    Alliance currentAlliance=DriverStation.getAlliance().get();
    Pose2d centerStartPos=autoBuilder.getStartingPose(AutoConstants.StartingLocations.CENTER1,currentAlliance);
    Pose2d centerNotePos=autoBuilder.getNotePose(AutoConstants.NotePoses.CENTERNOTEPOS,currentAlliance);
    Pose2d ampNotePos=autoBuilder.getNotePose(AutoConstants.NotePoses.AMPNOTEPOS,currentAlliance);
    Pose2d longAmpNotePos=autoBuilder.getNotePose(AutoConstants.NotePoses.LONGAMP1POS,currentAlliance);
    Pose2d longAmpIntakePos=autoBuilder.getNotePose(AutoConstants.NotePoses.LONGAMP1INTAKEPOS,currentAlliance);
    double turretAngle=autoBuilder.getTurretAngle(currentAlliance,AutoConstants.ShootLocations.AMPSHOOT);
    System.out.println("Turret Angle Set to +" + turretAngle);
    SmartShooter shoot1=new SmartShooter(m_shooter, m_turret, m_swerve, m_TurretLimelight, m_intake,false,false,false,false);
    SmartShooter shoot2=new SmartShooter(m_shooter, m_turret, m_swerve, m_TurretLimelight, m_intake,false,false,false,false);
    SmartShooter shoot3=new SmartShooter(m_shooter, m_turret, m_swerve, m_TurretLimelight, m_intake,false,false,false,false);

    TurretPosCommand turretSet00=new TurretPosCommand(m_turret,Constants.TurretConstants.turretStraightPos,Constants.TurretConstants.turretMoveTimeOut,Constants.TurretConstants.deadband);
    TurretPosCommand turretSetLeftShoot=new TurretPosCommand(m_turret,turretAngle,Constants.TurretConstants.turretMoveTimeOut,Constants.TurretConstants.deadband);
    TurretPosCommand turretSet01=new TurretPosCommand(m_turret,Constants.TurretConstants.turretStraightPos,Constants.TurretConstants.turretMoveTimeOut,Constants.TurretConstants.deadband);
    TurretPosCommand turretSet02=new TurretPosCommand(m_turret,Constants.TurretConstants.turretStraightPos,Constants.TurretConstants.turretMoveTimeOut,Constants.TurretConstants.deadband);
    TurretPosCommand turretSet90=new TurretPosCommand(m_turret,Constants.TurretConstants.turret90Pos,Constants.TurretConstants.turretMoveTimeOut,Constants.TurretConstants.deadband);

    IntakeToggleCommand intake1=new IntakeToggleCommand(m_intake,IntakeConstants.autoIntakeSpeed,true);
    IntakeToggleCommand intake2=new IntakeToggleCommand(m_intake,IntakeConstants.autoIntakeSpeed,true);
    IntakeToggleCommand intake3=new IntakeToggleCommand(m_intake,IntakeConstants.autoIntakeSpeed,true);
  
    ShootRotateSetReferenceCommand shooterSetRef1=new ShootRotateSetReferenceCommand(m_shooter,0);
    ShootRotateSetReferenceCommand shooterSetRef2=new ShootRotateSetReferenceCommand(m_shooter,0);
    ShootRotateSetReferenceCommand shooterSetRef3=new ShootRotateSetReferenceCommand(m_shooter,0);

    Command startToCenterNote=m_swerve.createPathCommand(autoBuilder.CreateAutoPath(centerStartPos,centerNotePos));
    Command centerNoteToAmpNote=m_swerve.createPathCommand(autoBuilder.CreateAutoPath(centerNotePos,ampNotePos));
    Command centerNoteLongAmpIntake=m_swerve.createPathCommand(autoBuilder.CreateAutoPath(ampNotePos,longAmpNotePos));
    Command longAmpIntakeToLongAmpPos=m_swerve.createPathCommand(autoBuilder.CreateAutoPath(longAmpIntakePos,longAmpNotePos));

    ParallelCommandGroup newMoveSetRef1=new ParallelCommandGroup(turretSet00,startToCenterNote,shooterSetRef1);
    ParallelCommandGroup newMoveSetRef2=new ParallelCommandGroup(turretSet90,centerNoteToAmpNote,shooterSetRef2);
    //ParallelCommandGroup newMoveSetRef3=new ParallelCommandGroup(turretSet01,ampNoteToCenterNote);
    ParallelCommandGroup newMoveSetRef4=new ParallelCommandGroup(turretSet01,centerNoteLongAmpIntake,shooterSetRef3);

    m_swerve.resetOdometry(centerStartPos);
    SequentialCommandGroup newSeq=new SequentialCommandGroup(shoot1,intake1,newMoveSetRef1,shoot2,
                                                            intake2,newMoveSetRef2,turretSetLeftShoot,
                                                            shoot3,intake3,newMoveSetRef4);

    return newSeq;
  }

  public Command PathPlanRightScoreLongSource1LongSource2(){
    ChooseYourOwnAdventureAuto autoBuilder=new ChooseYourOwnAdventureAuto(m_swerve, m_shooter, m_intake, m_turret, m_TurretLimelight);
    Alliance currentAlliance=DriverStation.getAlliance().get();
    Pose2d rightStartPos=autoBuilder.getStartingPose(AutoConstants.StartingLocations.RIGHTSPEAKER,currentAlliance);
    Pose2d longSource1NotePos=autoBuilder.getNotePose(AutoConstants.NotePoses.LONGSOURCE1POS,currentAlliance);
    Pose2d longSource1IntakeWaypoint=autoBuilder.getNotePose(AutoConstants.NotePoses.LONGSOURCE1WAYPOINT,currentAlliance);
    Pose2d longSource2NotePos=autoBuilder.getNotePose(AutoConstants.NotePoses.LONGSOURCE2POS,currentAlliance);
    double startTurretPos=autoBuilder.getTurretAngle(currentAlliance,AutoConstants.ShootLocations.RIGHTSPEAKER);
    double wingShootTurretPos=autoBuilder.getTurretAngle(currentAlliance,AutoConstants.ShootLocations.RIGHTWINGSHOOTPOS);
    Pose2d shootPos=autoBuilder.getNotePose(AutoConstants.NotePoses.WINGSHOOTPOS, currentAlliance);

    ShootCommand shoot1=new ShootCommand(m_shooter, m_intake,AutoConstants.autoCloseSpeed1,AutoConstants.autoCloseSpeed2,AutoConstants.autoCloseAngle,true,false);
    ShootCommand shoot2=new ShootCommand(m_shooter, m_intake,AutoConstants.wingShotSpeed1,AutoConstants.wingShotSpeed2,AutoConstants.wingShotAngle,true,false);
    SmartShooter shoot3=new SmartShooter(m_shooter, m_turret, m_swerve, m_TurretLimelight, m_intake,false,false,false,false);

    ShooterNotePass m_primeMotors=new ShooterNotePass(m_shooter, m_turret, m_intake,AutoConstants.wingShotSpeed1,AutoConstants.wingShotSpeed2,0,true);

    TurretPosCommand turretSet00=new TurretPosCommand(m_turret,Constants.TurretConstants.turretStraightPos,Constants.TurretConstants.turretMoveTimeOut,Constants.TurretConstants.deadband);
    TurretPosCommand turretSet90=new TurretPosCommand(m_turret,Constants.TurretConstants.turret90Pos,Constants.TurretConstants.turretMoveTimeOut,Constants.TurretConstants.deadband);
    TurretPosCommand turretSetRightShoot=new TurretPosCommand(m_turret,startTurretPos,Constants.TurretConstants.turretMoveTimeOut,Constants.TurretConstants.deadband);
    TurretPosCommand turretSetRightWingShoot1=new TurretPosCommand(m_turret,wingShootTurretPos,Constants.TurretConstants.turretMoveTimeOut,Constants.TurretConstants.deadband);
    TurretPosCommand turretSetRightWingShoot2=new TurretPosCommand(m_turret,wingShootTurretPos,Constants.TurretConstants.turretMoveTimeOut,Constants.TurretConstants.deadband);

    IntakeToggleCommand intake1=new IntakeToggleCommand(m_intake,IntakeConstants.autoIntakeSpeed,true);
    IntakeToggleCommand intake2=new IntakeToggleCommand(m_intake,IntakeConstants.autoIntakeSpeed,true);
  
    ShootRotateSetReferenceCommand shooterSetRef1=new ShootRotateSetReferenceCommand(m_shooter,0);
    ShootRotateSetReferenceCommand shooterSetRef2=new ShootRotateSetReferenceCommand(m_shooter,0);

    Command startToShootPos=m_swerve.createPathCommand(autoBuilder.CreateAutoPath(rightStartPos,longSource1IntakeWaypoint,longSource1NotePos,shootPos));
    Command shotPosToSource2ToShotPos=m_swerve.createPathCommand(autoBuilder.CreateAutoPath(shootPos,longSource1NotePos,longSource2NotePos,longSource1NotePos,shootPos));

    ParallelCommandGroup newMoveSetRef1=new ParallelCommandGroup(turretSet00,shooterSetRef1,startToShootPos);
    ParallelCommandGroup newMoveSetRef2=new ParallelCommandGroup(shotPosToSource2ToShotPos,turretSet90);

    m_swerve.resetOdometry(rightStartPos);

    SequentialCommandGroup newSeq=new SequentialCommandGroup(turretSetRightShoot,shoot1,intake1,newMoveSetRef1,
                                                            turretSetRightWingShoot1,shoot2);//,intake2,newMoveSetRef2,
                                                            //turretSetRightWingShoot2,shoot3);
    return newSeq;
  }
}    
