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
    
    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;
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

    private Swerve m_swerve = new Swerve(new SwerveIO(){});
//    private LED m_led=new LED(Constants.LEDConstants.blinkInPWM,Constants.LEDConstants.ledPWM,Constants.LEDConstants.ledCount);
    //private LED m_led= new LED(Constants.LEDConstants.ledPWM,Constants.LEDConstants.ledCount);
    private Blinkin m_blinkin=new Blinkin(Constants.BlinkinConstants.pwmID);
    private DigitalInput m_brakeButton = new DigitalInput(Constants.brakeButton);
    private Limelight m_TurretLimelight = new Limelight(Constants.LimeLightValues.targetHeight,Constants.LimeLightValues.limelightTurretHeight,
                                        Constants.LimeLightValues.limelightTurretAngle);
    /*private Limelight m_StaticLimelight = new Limelight(Constants.LimeLightValues.targetHeight,Constants.LimeLightValues.limelightTurretHeight,
                                        Constants.LimeLightValues.limelightTurretAngle,0,0,Constants.LimeLightValues.staticLimelightName);*/
    private NoteFlipperSubsystem m_flip=new NoteFlipperSubsystem(Constants.NoteFlipConstants.canID);
    private TurretSubsystem m_turret=new TurretSubsystem(Constants.TurretConstants.turretMotorPort,
                                         Constants.TurretConstants.turretLimitSwitchPort,
                                         m_TurretLimelight,new TurretSubsystemIO(){});
    private IntakeSubsystem m_intake=new IntakeSubsystem(Constants.IntakeConstants.intakeMotorPort1,
                                         Constants.IntakeConstants.intakeMotorPort2,
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

      TurretCommand turretLeft=new TurretCommand(m_turret,Constants.TurretConstants.turretSpeed);
      POVButton turretLeftButton=new POVButton(m_xBoxDriver,Constants.ButtonBindings.driverturretPOVLeft);
      turretLeftButton.whileTrue(turretLeft);

      TurretCommand turretOperatorLeft=new TurretCommand(m_turret,Constants.TurretConstants.turretSpeed);
      POVButton turretOperatorLeftButton=new POVButton(m_xBoxOperator,Constants.ButtonBindings.operatorturretPOVLeft);
      turretOperatorLeftButton.whileTrue(turretOperatorLeft);

      TurretCommand turretRight=new TurretCommand(m_turret,-Constants.TurretConstants.turretSpeed);
      POVButton turretRightButton=new POVButton(m_xBoxDriver,Constants.ButtonBindings.driverturretPOVRight);
      turretRightButton.whileTrue(turretRight);

      
      TurretCommand turretOperatorRight=new TurretCommand(m_turret,-Constants.TurretConstants.turretSpeed);
      POVButton turretOperatorRightButton=new POVButton(m_xBoxOperator,Constants.ButtonBindings.operatorturretPOVRight);
      turretOperatorRightButton.whileTrue(turretOperatorRight);

      //ShootCommand shoot1=new ShootCommand(m_shooter,m_intake,Constants.ShooterConstants.testShooterSpeed1,Constants.ShooterConstants.baseMotorSpeed);
      //JoystickButton shootButton1=new JoystickButton(m_xBoxDriver,Constants.ButtonBindings.drivermanualShootButton);
      //shootButton1.toggleOnTrue(shoot1);

      SmartShooter smartToggleTurret=new SmartShooter(m_shooter, m_turret, m_swerve, m_TurretLimelight, m_intake, false,false,true,true);
      Trigger smartToggleTurretButton= new Trigger(() -> m_xBoxOperator.getRawAxis(leftTriggerAxis)>Constants.ButtonBindings.triggerDeadband);
      smartToggleTurretButton.toggleOnTrue(smartToggleTurret);

      SmartShooter smartShooter=new SmartShooter(m_shooter, m_turret, m_swerve, m_TurretLimelight, m_intake, false,false,true,false);
      Trigger shootOpTrigger= new Trigger(() -> m_xBoxOperator.getRawAxis(rightTriggerAxis)>Constants.ButtonBindings.triggerDeadband);
      shootOpTrigger.onTrue(smartShooter);

      ShootRotateSetReferenceCommand setref=new ShootRotateSetReferenceCommand(m_shooter,Constants.ShooterConstants.shortShotEncoderVal);
      JoystickButton setrefButton= new JoystickButton(m_xBoxOperator,Constants.ButtonBindings.operatorShooterIntake);
      setrefButton.whileTrue(setref);
      //Trigger setrefTrigger= new Trigger(() -> m_xBoxOperator.getRawAxis(rightTriggerAxis)>Constants.ButtonBindings.triggerDeadband);
      //setrefTrigger.whileTrue(setref);

      AmpScoreHopeCommand noteFlip=new AmpScoreHopeCommand(m_shooter,m_intake,m_flip,Constants.ShooterConstants.shooterAmpScoreAngle);
      JoystickButton noteFlipButton=new JoystickButton(m_xBoxOperator,ButtonBindings.operatorNoteFlip);
      noteFlipButton.whileTrue(noteFlip);

      ShootCommand shooterIntakeOp=new ShootCommand(m_shooter,m_intake,Constants.ShooterConstants.shooterIntakeSpeed,Constants.ShooterConstants.baseMotorSpeed);
      //JoystickButton shootIntakeOpButton= new JoystickButton(m_xBoxOperator,Constants.ButtonBindings.operatorShooterIntake);
      //shootIntakeOpButton.toggleOnTrue(shooterIntakeOp)

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

      ShootCommand shootCustom1=new ShootCommand(m_shooter,m_intake,Constants.customShot1Velocity1,Constants.customShot1Velocity2,Constants.customShot1Angle,true,false);
      JoystickButton custom1 = new JoystickButton(m_CustomController, Constants.ButtonBindings.customShot1);
      custom1.onTrue(shootCustom1);

      ShootCommand shootCustom2=new ShootCommand(m_shooter,m_intake,Constants.customShot2Velocity1,Constants.customShot2Velocity2,Constants.customShot2Angle,true,false);
      JoystickButton custom2 = new JoystickButton(m_CustomController, Constants.ButtonBindings.customShot2);
      custom2.onTrue(shootCustom2);

      ShootCommand shootCustom3=new ShootCommand(m_shooter,m_intake,Constants.customShot3Velocity1,Constants.customShot3Velocity2,Constants.customShot3Angle,true,false);
      JoystickButton custom3 = new JoystickButton(m_CustomController, Constants.ButtonBindings.customShot3);
      custom3.onTrue(shootCustom3);

      
      ShootCommand shootManual=new ShootCommand(m_shooter,m_intake,Constants.customShot1Velocity1,Constants.customShot1Velocity1,Constants.customShot3Angle,false,false);
      JoystickButton customManual = new JoystickButton(m_CustomController, Constants.ButtonBindings.customManual);
      customManual.onTrue(shootManual);

      //ShootCommand shootCustom4=new ShootCommand(m_shooter,m_intake,Constants.customShot4Velocity1,Constants.customShot4Velocity2,Constants.customShot4Angle,false);
      SourceIntakeCommand intakeCustom4=new SourceIntakeCommand(m_shooter,ShooterConstants.shooterSourceIntakeAngle);
      JoystickButton intakeCustom4Button = new JoystickButton(m_CustomController, Constants.ButtonBindings.customShot4);
      intakeCustom4Button.whileTrue(intakeCustom4);

      ShootRotateSetReferenceCommand test=new ShootRotateSetReferenceCommand(m_shooter,Constants.ShooterConstants.PIDTESTAngle);
      JoystickButton testButton=new JoystickButton(m_xBoxDriver,ButtonBindings.driverturret90);
      testButton.whileTrue(test);

    }
       
    private void refreshSmartDashboard(){  
      //TODO: ADD BACK WHEN LIMELIGHT ON
      //m_TurretLimelight.LimeLightPeriodic(true);
//      m_swerve.getPose();

      if(m_turret.IsAtLeftLimit()||m_turret.IsAtRightLimit()){
        m_blinkin.SetLEDPrimaryState(LEDStates.ISATLIMIT);
      }else{
        if(m_TurretLimelight.isTargetAvailible()){
          m_blinkin.SetLEDPrimaryState(LEDStates.TARGETLOCK);
        }else{
          m_blinkin.SetLEDPrimaryState(LEDStates.NOTARGET);
        }
      }
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
        m_autoChooser.addOption(AutoConstants.autoMode10,AutoConstants.autoMode10);
        m_autoChooser.addOption(AutoConstants.autoMode11,AutoConstants.autoMode11);

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
    switch (autoChosen) {
      case Constants.AutoConstants.autoMode1: // Do Nothing
        returnCommand=new AutoDoNothingCommand();
        break;
      case Constants.AutoConstants.autoMode2: // 2=PP-Start Center,Score, Center Short, Amp Short
        returnCommand=new AutoDoNothingCommand();
        break;
      case Constants.AutoConstants.autoMode3: // 3=PP-Start Left, Amp Short, Amp Long
        returnCommand=new AutoDoNothingCommand();
        break;
      case Constants.AutoConstants.autoMode4: // 4=PP-Start Right, Source Long
        returnCommand=new AutoDoNothingCommand();
        break;
      case Constants.AutoConstants.autoMode7: // Legacy-Score, Straight Back Score
        returnCommand=legacyAutoShootMoveBackShoot();
        break;
      case Constants.AutoConstants.autoMode8: // Legacy-Score,Wait, Cross the Line Right
        returnCommand=new AutoDoNothingCommand();
        break;
      case Constants.AutoConstants.autoMode9: // Legacy-Score, Center Amp Short, Amp Short
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
  } 
  
  public void AllPeriodic(){
       refreshSmartDashboard();
        m_TurretLimelight.LimeLightPeriodic(true); 
  }

  public void TeleopPeriodic(){
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
    homeRobot();

    ChangeVisionPipeline();
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
    TurretPosCommand turretSet0=new TurretPosCommand(m_turret,Constants.TurretConstants.turretStraightPos,Constants.TurretConstants.turretMoveTimeOut,Constants.TurretConstants.deadband);
    IntakeToggleCommand startIntake1=new IntakeToggleCommand(m_intake,Constants.IntakeConstants.intakeSpeed,true);
    AutoMoveCommand moveback = new AutoMoveCommand(m_swerve,0.0, Constants.AutoConstants.distancetoShortNote);
    return new SequentialCommandGroup(shoot0,turretSet0,startIntake1,moveback,shoot1);
  }
    private Command legacyAutoShootMoveBackShoot(Alliance currentAlliance){
    SmartShooter shoot0=new SmartShooter(m_shooter, m_turret, m_swerve, m_TurretLimelight, m_intake, false,false,true,false); 
    SmartShooter shoot1=new SmartShooter(m_shooter, m_turret, m_swerve, m_TurretLimelight, m_intake, false,false,true,false); 
    SmartShooter shoot2=new SmartShooter(m_shooter, m_turret, m_swerve, m_TurretLimelight, m_intake, false,false,true,false); 
    TurretPosCommand turretSet0=new TurretPosCommand(m_turret,Constants.TurretConstants.turretStraightPos,Constants.TurretConstants.turretMoveTimeOut,Constants.TurretConstants.deadband);
    TurretPosCommand turretSet90=new TurretPosCommand(m_turret,Constants.TurretConstants.turret90Pos,Constants.TurretConstants.turretMoveTimeOut,Constants.TurretConstants.deadband);
    IntakeToggleCommand startIntake1=new IntakeToggleCommand(m_intake,Constants.IntakeConstants.intakeSpeed,true);
    IntakeToggleCommand startIntake2=new IntakeToggleCommand(m_intake,Constants.IntakeConstants.intakeSpeed,true);
    AutoMoveCommand moveback = new AutoMoveCommand(m_swerve,0.0, Constants.AutoConstants.distancetoShortNote);
    AutoMoveCommand moveover;
    AutoMoveCommand moveover2;
    if(currentAlliance==Alliance.Red){ 
      moveover = new AutoMoveCommand(m_swerve,0.0, Constants.AutoConstants.distanceBetweeNotes);
      moveover2 = new AutoMoveCommand(m_swerve,0.0, -Constants.AutoConstants.distanceBetweeNotes);
    }else { // switch direction for Blue
      moveover = new AutoMoveCommand(m_swerve,0.0, -Constants.AutoConstants.distanceBetweeNotes);
      moveover2 = new AutoMoveCommand(m_swerve,0.0, Constants.AutoConstants.distanceBetweeNotes);

    }
      return new SequentialCommandGroup(shoot0,turretSet0,startIntake1,moveback,shoot1,turretSet90,startIntake2,moveover,moveover2,shoot2);
  }
  }
}    
