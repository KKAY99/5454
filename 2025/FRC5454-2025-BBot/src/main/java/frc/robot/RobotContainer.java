package frc.robot;
import com.ctre.phoenix6.Utils;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.*;
import frc.robot.Constants.*;
import edu.wpi.first.wpilibj2.command.button.POVButton;

public class RobotContainer {
  private final Field2d m_Field2d = new Field2d();
  
  private final CommandXboxController m_xBoxDriver = new CommandXboxController(InputControllers.kXboxDrive);

  public final CommandSwerveDrivetrain m_swerve = TunerConstants.createDrivetrain();    
    // The robot's subsystems and commands are defined here...
    private ClimbSubsystem m_Climb = new ClimbSubsystem(61);
    private IntakeSubsystem m_Intake =  new IntakeSubsystem(60, 62, 1);
    
    // Dashboard inputs
    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;
 

  public boolean m_hasResetGyro=false;

  private final SendableChooser<Command> m_autoChooser = new SendableChooser<>();


  public RobotContainer(){
    
    NamedCommands.registerCommand("PlaceL1", m_Intake.score());
    m_autoChooser.setDefaultOption("center L1", new PathPlannerAuto("Trough", true).withName("Trough"));
    m_autoChooser.addOption("processer 2.5 coral", new PathPlannerAuto("processer 2.5 coral", true).withName("processer 2.5 coral"));
    m_autoChooser.addOption("2.5 coral push", new PathPlannerAuto("2.5 coral push", true).withName("2.5 coral push"));
    configureButtonBindings();
    resetDefaultCommand();
  }

  public Command getAutoRoutine(){
    return m_autoChooser.getSelected();
  }

  private void configureButtonBindings(){
    GasPedalCommand gasPedalCommand = new GasPedalCommand(m_swerve, ()->m_xBoxDriver.getRightTriggerAxis());
    m_xBoxDriver.rightTrigger().whileTrue(gasPedalCommand);

    ClimbCommand runclimbCommand = new ClimbCommand(m_Climb, Constants.climbSpeed);
    m_xBoxDriver.button(ButtonConstants.ClimbUp).whileTrue(runclimbCommand);

    ClimbCommand runclimbCommandPOVUp = new ClimbCommand(m_Climb, Constants.climbSpeed);
    m_xBoxDriver.pov(ButtonConstants.POVClimbUp).whileTrue(runclimbCommandPOVUp);

    ClimbCommand runclimbCommandPOVDown = new ClimbCommand(m_Climb, Constants.climbDownSpeed);
    m_xBoxDriver.pov(ButtonConstants.POVClimbDown).whileTrue(runclimbCommandPOVDown);

    IntakeCommand runIntakeCommand = new IntakeCommand(m_Intake,Constants.intakeInSpeed);
    m_xBoxDriver.button(ButtonConstants.DriverIntakeIn).whileTrue(runIntakeCommand);

    
    IntakeCommand runIntakeOutCommand = new IntakeCommand(m_Intake,Constants.intakeOutSpeed);
    m_xBoxDriver.button(ButtonConstants.DriverOutake).whileTrue(runIntakeOutCommand);

    IntakeRotateCommand runRotateCommand = new IntakeRotateCommand(m_Intake, Constants.rotateUpSpeed);
    m_xBoxDriver.button(ButtonConstants.RotateUp).whileTrue(runRotateCommand);
    
    IntakeRotateCommand runRotateDownCommand = new IntakeRotateCommand(m_Intake, Constants.rotateDownSpeed);
    m_xBoxDriver.button(Constants.ButtonConstants.RotateDown).whileTrue(runRotateDownCommand);

    zAutoIntakeCommand autoIntake = new zAutoIntakeCommand(m_Intake,AutomationConstants.autoIntakeTargetPos,AutomationConstants.autoIntakeSpeed);
    m_xBoxDriver.button(ButtonConstants.AutoIntake).whileTrue(autoIntake);

    zAutoIntakeCommand autoOutake = new zAutoIntakeCommand(m_Intake,AutomationConstants.autoOutakeTargetPos,AutomationConstants.autoOuttakeSpeed);
    m_xBoxDriver.button(ButtonConstants.AutoOutake).whileTrue(autoOutake);
}



  public void AutonMode(){
    
  }

  public void TeleopMode(){
    
  }
  
  public void TeleopPeriodic(){
    System.out.println("Rotate Position" + m_Intake.getRotatePosition() + "ABS Position" + m_Intake.getRotatePositionABS());
  }

  public void AllPeriodic(){
    m_Field2d.setRobotPose(m_swerve.getPose2d());
  }

  public void AutoPeriodic(){
    } 

    public void DisabledInit(){
  }

  public void DisabledPeriodic(){
    
  }

  private void resetDefaultCommand(){
    m_swerve.setDefaultCommand(m_swerve.applyRequestDrive(m_xBoxDriver,translationAxis,strafeAxis,rotationAxis));
  }

  }
  
  