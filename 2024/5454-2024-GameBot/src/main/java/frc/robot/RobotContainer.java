// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.sql.Driver;
import java.util.List;

import javax.swing.JToggleButton;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import frc.robot.utilities.Lasercan;
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
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants.InputControllers;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.ButtonBindings;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.utilities.ADABreakBeam;
import frc.robot.utilities.AutoCommands;
import frc.robot.utilities.LED;
import frc.robot.utilities.Limelight;
import frc.robot.utilities.ModularAutoBuilder;



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
 
    private XboxController m_xBoxDriver = new XboxController(InputControllers.kXboxDrive);
    private SendableChooser<String> m_autoChooser = new SendableChooser<>(); 
    private SendableChooser<AutoConstants.StartingLocations> m_autoStart = new SendableChooser<>(); 
    private SendableChooser<Double> m_autoDelay = new SendableChooser<>(); 
    private SendableChooser<Boolean> m_shouldUseModularBuilder = new SendableChooser<>();
    private SendableChooser<Boolean> m_shootFinalNote = new SendableChooser<>();
    private SendableChooser<Pose2d> m_autoPath1 = new SendableChooser<>();
    private SendableChooser<Pose2d> m_autoPath2 = new SendableChooser<>(); 
    private SendableChooser<Pose2d> m_autoPath3 = new SendableChooser<>(); 
    private SendableChooser<Pose2d> m_autoPath4 = new SendableChooser<>(); 
    private SendableChooser<Pose2d> m_autoPath5 = new SendableChooser<>(); 

    private Swerve m_swerve = new Swerve(new SwerveIO(){});
    private LED m_led=new LED(Constants.LEDConstants.blinkInPWM,Constants.LEDConstants.ledPWM,Constants.LEDConstants.ledCount);
    private DigitalInput m_brakeButton = new DigitalInput(Constants.brakeButton);
    private TurretSubsystem m_turret=new TurretSubsystem(Constants.TurretConstants.turretMotorPort,
                                         Constants.TurretConstants.turretLimitSwitchPort,new TurretSubsystemIO(){});
    private IntakeSubsystem m_intake=new IntakeSubsystem(Constants.IntakeConstants.intakeMotorPort1,
                                         Constants.IntakeConstants.intakeMotorPort2,new IntakeSubsystemIO(){});
    private ClimbSubsystem m_climb = new ClimbSubsystem(Constants.climbConstants.climbPort);
    private Limelight m_Limelight = new Limelight(Constants.LimeLightValues.targetHeight,Constants.LimeLightValues.limelightHeight,
                                        Constants.LimeLightValues.limelightAngle);
    private ShooterSubsystem m_shooter=new ShooterSubsystem(m_Limelight,Constants.ShooterConstants.shooterMotorPort1,
                                           Constants.ShooterConstants.shooterMotorPort2,Constants.ShooterConstants.shooterAnglePort);

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
        m_swerve.setDefaultCommand(
        m_swerve.drive(
            () -> m_xBoxDriver.getRawAxis(translationAxis),
            () -> m_xBoxDriver.getRawAxis(strafeAxis),
            () -> m_xBoxDriver.getRawAxis(rotationAxis),
            () -> m_xBoxDriver.getRawAxis(rightTriggerAxis)));

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
      //JoystickButton intakeToggleTrueButtonIn=new JoystickButton(m_xBoxDriver,ButtonBindings.intakeToggleButtonIn);
      //intakeToggleTrueButtonIn.onTrue(intakeToggleTrueIn);

      IntakeToggleCommand intakeToggleTrueOut=new IntakeToggleCommand(m_intake,-Constants.IntakeConstants.intakeSpeed);
      JoystickButton intakeToggleTrueButtonOut=new JoystickButton(m_xBoxDriver,ButtonBindings.intakeToggleButtonOut);
      intakeToggleTrueButtonOut.onTrue(intakeToggleTrueOut);

      IntakeConveyCommand intakeConvey=new IntakeConveyCommand(m_intake,Constants.IntakeConstants.intakeBreakBeamport,m_led);
      JoystickButton intakeConveyButton=new JoystickButton(m_xBoxDriver,ButtonBindings.intakeToggleButtonIn);
      intakeConveyButton.toggleOnTrue(intakeConvey);

      TurretCommand turretLeft=new TurretCommand(m_turret,Constants.TurretConstants.turretSpeed);
      POVButton turretLeftButton=new POVButton(m_xBoxDriver,Constants.ButtonBindings.turretPOVLeft);
      turretLeftButton.whileTrue(turretLeft);

      TurretCommand turretRight=new TurretCommand(m_turret,-Constants.TurretConstants.turretSpeed);
      POVButton turretRightButton=new POVButton(m_xBoxDriver,Constants.ButtonBindings.turretPOVRight);
      turretRightButton.whileTrue(turretRight);

      TurretPosCommand turretStraight=new TurretPosCommand(m_turret,Constants.TurretConstants.turretStraightPos,
                                                          Constants.TurretConstants.turretMoveTimeOut,Constants.TurretConstants.deadband);
      JoystickButton turretStraightButton=new JoystickButton(m_xBoxDriver,Constants.ButtonBindings.turret0);
      turretStraightButton.onTrue(turretStraight);

      TurretPosCommand turret90=new TurretPosCommand(m_turret,Constants.TurretConstants.turret90Pos,
                                                    Constants.TurretConstants.turretMoveTimeOut,Constants.TurretConstants.deadband);
      JoystickButton turret90Button=new JoystickButton(m_xBoxDriver,Constants.ButtonBindings.turret90);
      turret90Button.onTrue(turret90);

      /*ShootCommand shoot1=new ShootCommand(m_shooter,Constants.ShooterConstants.testShooterSpeed1,Constants.ShooterConstants.baseMotorSpeed);
      JoystickButton shootButton1=new JoystickButton(m_xBoxDriver,Constants.ButtonBindings.manualShootButton);
      shootButton1.whileTrue(shoot1);

      /*ShootCommand shoot2=new ShootCommand(m_shooter,Constants.ShooterConstants.testShooterSpeed2);
      JoystickButton shootButton2=new JoystickButton(m_xBoxDriver,Constants.ButtonBindings.testShooter2Button);
      shootButton2.whileTrue(shoot2);

      /*RobotTrackCommand turretTrack=new RobotTrackCommand(m_Limelight,m_turret);
      JoystickButton turretTrackButton=new JoystickButton(m_xBoxDriver,Constants.ButtonBindings.testShooter1Button);
      turretTrackButton.whileTrue(turretTrack);*/
    }
       
    private void refreshSmartDashboard(){  
      //TODO: ADD BACK WHEN LIMELIGHT ON
      //m_Limelight.update(true);
      m_swerve.getPose();
    }
    
    

   /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */

  private void createAutonomousCommandList(){
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
      SmartDashboard.putData("Auto Start",m_autoStart);

      m_autoPath1.setDefaultOption(AutoConstants.noPath, null);
      m_autoPath1.addOption(AutoConstants.blueShortAmpNote,AutoConstants.locationBlueShortAmpNote);
      m_autoPath1.addOption(AutoConstants.blueShortCenterNote,AutoConstants.locationBlueShortCenterNote);
      m_autoPath1.addOption(AutoConstants.blueShortSourceNote,AutoConstants.locationBlueShortSourceNote);
      m_autoPath1.addOption(AutoConstants.redShortAmpNote,AutoConstants.locationRedShortAmpNote);
      m_autoPath1.addOption(AutoConstants.redShortCenterNote,AutoConstants.locationRedShortCenterNote);
      m_autoPath1.addOption(AutoConstants.redShortSourceNote,AutoConstants.locationRedShortSourceNote);
      m_autoPath1.addOption(AutoConstants.longAmpNote1,AutoConstants.locationLongAmpNote);
      m_autoPath1.addOption(AutoConstants.longAmpNote2,AutoConstants.locationLongAmp2Note);
      m_autoPath1.addOption(AutoConstants.longCenterNote,AutoConstants.locationLongCenterNote);
      m_autoPath1.addOption(AutoConstants.longSourceNote1,AutoConstants.locationLongSourceNote);
      m_autoPath1.addOption(AutoConstants.longSourceNote2,AutoConstants.locationLongSource2Note);
      SmartDashboard.putData("Auto Path 1",m_autoPath1);

      m_autoPath2.setDefaultOption(AutoConstants.noPath, null);
      m_autoPath2.addOption(AutoConstants.blueShortAmpNote,AutoConstants.locationBlueShortAmpNote);
      m_autoPath2.addOption(AutoConstants.blueShortCenterNote,AutoConstants.locationBlueShortCenterNote);
      m_autoPath2.addOption(AutoConstants.blueShortSourceNote,AutoConstants.locationBlueShortSourceNote);
      m_autoPath2.addOption(AutoConstants.redShortAmpNote,AutoConstants.locationRedShortAmpNote);
      m_autoPath2.addOption(AutoConstants.redShortCenterNote,AutoConstants.locationRedShortCenterNote);
      m_autoPath2.addOption(AutoConstants.redShortSourceNote,AutoConstants.locationRedShortSourceNote);
      m_autoPath2.addOption(AutoConstants.longAmpNote1,AutoConstants.locationLongAmpNote);
      m_autoPath2.addOption(AutoConstants.longAmpNote2,AutoConstants.locationLongAmp2Note);
      m_autoPath2.addOption(AutoConstants.longCenterNote,AutoConstants.locationLongCenterNote);
      m_autoPath2.addOption(AutoConstants.longSourceNote1,AutoConstants.locationLongSourceNote);
      m_autoPath2.addOption(AutoConstants.longSourceNote2,AutoConstants.locationLongSource2Note);
      SmartDashboard.putData("Auto Path 2",m_autoPath2);

      m_autoPath3.setDefaultOption(AutoConstants.noPath, null);
      m_autoPath3.addOption(AutoConstants.blueShortAmpNote,AutoConstants.locationBlueShortAmpNote);
      m_autoPath3.addOption(AutoConstants.blueShortCenterNote,AutoConstants.locationBlueShortCenterNote);
      m_autoPath3.addOption(AutoConstants.blueShortSourceNote,AutoConstants.locationBlueShortSourceNote);
      m_autoPath3.addOption(AutoConstants.redShortAmpNote,AutoConstants.locationRedShortAmpNote);
      m_autoPath3.addOption(AutoConstants.redShortCenterNote,AutoConstants.locationRedShortCenterNote);
      m_autoPath3.addOption(AutoConstants.redShortSourceNote,AutoConstants.locationRedShortSourceNote);
      m_autoPath3.addOption(AutoConstants.longAmpNote1,AutoConstants.locationLongAmpNote);
      m_autoPath3.addOption(AutoConstants.longAmpNote2,AutoConstants.locationLongAmp2Note);
      m_autoPath3.addOption(AutoConstants.longCenterNote,AutoConstants.locationLongCenterNote);
      m_autoPath3.addOption(AutoConstants.longSourceNote1,AutoConstants.locationLongSourceNote);
      m_autoPath3.addOption(AutoConstants.longSourceNote2,AutoConstants.locationLongSource2Note);
      SmartDashboard.putData("Auto Path 3",m_autoPath3);

      m_autoPath4.setDefaultOption(AutoConstants.noPath, null);
      m_autoPath4.addOption(AutoConstants.blueShortAmpNote,AutoConstants.locationBlueShortAmpNote);
      m_autoPath4.addOption(AutoConstants.blueShortCenterNote,AutoConstants.locationBlueShortCenterNote);
      m_autoPath4.addOption(AutoConstants.blueShortSourceNote,AutoConstants.locationBlueShortSourceNote);
      m_autoPath4.addOption(AutoConstants.redShortAmpNote,AutoConstants.locationRedShortAmpNote);
      m_autoPath4.addOption(AutoConstants.redShortCenterNote,AutoConstants.locationRedShortCenterNote);
      m_autoPath4.addOption(AutoConstants.redShortSourceNote,AutoConstants.locationRedShortSourceNote);
      m_autoPath4.addOption(AutoConstants.longAmpNote1,AutoConstants.locationLongAmpNote);
      m_autoPath4.addOption(AutoConstants.longAmpNote2,AutoConstants.locationLongAmp2Note);
      m_autoPath4.addOption(AutoConstants.longCenterNote,AutoConstants.locationLongCenterNote);
      m_autoPath4.addOption(AutoConstants.longSourceNote1,AutoConstants.locationLongSourceNote);
      m_autoPath4.addOption(AutoConstants.longSourceNote2,AutoConstants.locationLongSource2Note);
      SmartDashboard.putData("Auto Path 4",m_autoPath4);

      m_autoPath5.setDefaultOption(AutoConstants.noPath, null);
      m_autoPath5.addOption(AutoConstants.blueShortAmpNote,AutoConstants.locationBlueShortAmpNote);
      m_autoPath5.addOption(AutoConstants.blueShortCenterNote,AutoConstants.locationBlueShortCenterNote);
      m_autoPath5.addOption(AutoConstants.blueShortSourceNote,AutoConstants.locationBlueShortSourceNote);
      m_autoPath5.addOption(AutoConstants.redShortAmpNote,AutoConstants.locationRedShortAmpNote);
      m_autoPath5.addOption(AutoConstants.redShortCenterNote,AutoConstants.locationRedShortCenterNote);
      m_autoPath5.addOption(AutoConstants.redShortSourceNote,AutoConstants.locationRedShortSourceNote);
      m_autoPath5.addOption(AutoConstants.longAmpNote1,AutoConstants.locationLongAmpNote);
      m_autoPath5.addOption(AutoConstants.longAmpNote2,AutoConstants.locationLongAmp2Note);
      m_autoPath5.addOption(AutoConstants.longCenterNote,AutoConstants.locationLongCenterNote);
      m_autoPath5.addOption(AutoConstants.longSourceNote1,AutoConstants.locationLongSourceNote);
      m_autoPath5.addOption(AutoConstants.longSourceNote2,AutoConstants.locationLongSource2Note);
      SmartDashboard.putData("Auto Path 5",m_autoPath5);

      m_shootFinalNote.setDefaultOption("Dont Shoot Final Note",false);
      m_shootFinalNote.addOption("Shoot Final Note",true);
      SmartDashboard.putData("Should Shoot Final Note",m_shootFinalNote);

      m_shouldUseModularBuilder.setDefaultOption("Dont Use Modular Builder",false);
      m_shouldUseModularBuilder.addOption("Use Modular Builder",true);
      SmartDashboard.putData("Should Use Modular Builder",m_shouldUseModularBuilder);
  }

  public Command getAutonomousCommand(){
    Alliance currentAlliance=DriverStation.getAlliance().get();
    AutoCommands autoMaker = new AutoCommands(m_swerve,m_shooter,m_intake,m_turret,m_Limelight);
    ModularAutoBuilder autoModularMaker = new ModularAutoBuilder(m_swerve,m_shooter,m_intake,m_turret,m_Limelight);
    AutoConstants.StartingLocations startLocation=m_autoStart.getSelected();
    double delay=m_autoDelay.getSelected();
    String autoChosen=m_autoChooser.getSelected();
    Command newCommand=null;
    
    if(m_Limelight.isTargetAvailible()){
      m_swerve.resetOdometry(m_Limelight.GetPoseViaApriltag());
    }else{
      m_swerve.resetOdometry(autoMaker.getStartingPose(startLocation,currentAlliance));
    }

    if(m_shouldUseModularBuilder.getSelected()){
      newCommand=autoModularMaker.CreateAutoCommand(m_autoPath1.getSelected(),m_autoPath2.getSelected(),m_autoPath3.getSelected(),m_autoPath4.getSelected(),
                                                    m_autoPath5.getSelected(),m_shootFinalNote.getSelected());                                     
    }else{
      newCommand=autoMaker.createAutoCommand(startLocation,autoChosen,delay,currentAlliance);
    }

    return newCommand;
  }

  public void checkBrakeButton(){
    if(m_brakeButton.get() && m_brakeButtonPressed==false){ 
      m_brakeButtonPressed=true;
      if(m_isBrakeButtonToggled==false){
                    disableBrakeMode();
                    m_isBrakeButtonToggled=true;
            }else{
                    resetBrakeModetoNormal();
                    m_isBrakeButtonToggled=false;
            }
  }else{

    if(m_brakeButton.get()==false){
            m_brakeButtonPressed=false;
    }
  }
  }

  public void SetBaseShooterSpeed(){
    //m_shooter.RunShootingMotors(Constants.ShooterConstants.baseMotorSpeed);
  }

  public void AutonMode(){
  // m_ledStrip.setRobotMode(LEDSChargedup.LEDMode.AUTOMODE);
  // enableLimelights();
    resetBrakeModetoNormal();
    SetBaseShooterSpeed();
    homeRobot();  
  } 
  
  public void AllPeriodic(){
       refreshSmartDashboard();
  }

  public void TeleopPeriodic(){
   
  }

  public void AutoPeriodic(){
  
    if(m_Limelight.isTargetAvailible()){
    //Get Limelight
    //add Vision measurement to swerve
    //m_swerve.AddVisionPose(m_Limelight.GetPoseViaApriltag(),0,true,1);
    }
   
  }
  public void TeleopMode(){
    //m_ledStrip.setRobotMode(LEDSChargedup.LEDMode.TELEOP);
    resetBrakeModetoNormal();
    m_swerve.resetOdometry(Constants.AutoConstants.redCenterStartPos);
    SetBaseShooterSpeed();
    homeRobot();
  }
  private void resetBrakeModetoNormal(){
    m_intake.setBrakeOn();
 //   m_shooter.setBrakeOn();
    m_turret.setBrakeOn();
 //   m_climb.setBrakeOn();
  }

  private void disableBrakeMode(){
    m_intake.setCoastOn();
//    m_shooter.setCoastOn();
    m_turret.setCoastOn();
//    m_climb.setCoastOn();
  }

  private void homeRobot(){
    if(m_HasHomed==false){
      Command turretHome=new TurretHomeCommand(m_turret);
      CommandScheduler.getInstance().schedule(turretHome);
      m_HasHomed=true;
    }
  }
}    
