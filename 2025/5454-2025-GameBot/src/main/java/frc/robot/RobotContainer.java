package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.*;
import frc.robot.utilities.AutoPlanner;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.*;
import com.ctre.phoenix6.StatusSignal.SignalMeasurement;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.InputControllers;

//@Logged(strategy=Strategy.OPT_IN)
public class RobotContainer {
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

  private final CommandXboxController m_xBoxDriver = new CommandXboxController(InputControllers.kXboxDrive); 
  private XboxController m_xBoxOperator = new XboxController(InputControllers.kXboxOperator);
  private Joystick m_CustomController = new Joystick(InputControllers.kCustomController);

  private SendableChooser<String> m_autoChooser = new SendableChooser<>(); 
  private SendableChooser<String> m_autoStartPos = new SendableChooser<>(); 

  public final CommandSwerveDrivetrain m_swerve = TunerConstants.createDrivetrain();

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
          .withDeadband(TunerConstants.kMaxSpeed * 0.1).withRotationalDeadband(TunerConstants.kMaxAngularSpeed*0.1); // Add a 10% deadband // Use open-loop control for drive motors
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

  private boolean isInTeleop=false;
  private Command autocommand = null;

  public RobotContainer(){
    //Named Commands
    NamedCommands.registerCommand("autoscore",new AutoDoNothingCommand()); 
    // Configure the button bindings
    configureButtonBindings();
    //Create Auto Commands
    createAutonomousCommandList(); 
    resetDefaultCommand();
      
  }

  private void configureButtonBindings() {

  }
      
  private void refreshSmartDashboard(){  
  }
  
  private boolean checkCANConnections(){
    return m_swerve.checkCANConnections();
  }

  private void createAutonomousCommandList(){
    try{
      m_autoChooser.setDefaultOption(AutoConstants.autoMode1,AutoConstants.autoMode1);
      m_autoChooser.addOption(AutoConstants.autoMode2,AutoConstants.autoMode2);

      SmartDashboard.putData("Auto Chooser",m_autoChooser);

    } catch (Exception e){
        System.out.println("Create Autos Failed, Exception: " + e.getMessage());
    }
  }

  public void DisabledInit(){
    
  }
   
  public void DisabledPeriodic(){
    
     
  }
  
  public void AutoPeriodic(){
  }

  public void AutonMode(){
   
  }

  public void TeleopMode(){
    System.out.println("autoo is" + CommandScheduler.getInstance().isScheduled(autocommand));
  }

  public void TeleopPeriodic(){
  }

  public void AllPeriodic(){
   
  }

  public Command getAutonomousCommand(){
    String autoChosen=m_autoChooser.getSelected();
    Command command=getSelectedAuto(autoChosen);
    
    return command;
  }

  public Command getSelectedAuto(String chosenAuto){
    AutoPlanner autoPlan=new AutoPlanner();
    Command command=null;
    
    switch(chosenAuto){
      case AutoConstants.autoMode1:
      command=new AutoDoNothingCommand();
      break;
      case AutoConstants.autoMode2:
      command=new SequentialCommandGroup(m_swerve.createPathCommand(
        autoPlan.CreateAutoPath(0,AutoConstants.AutoPoses.testPose1,AutoConstants.AutoPoses.testPose2,
                                AutoConstants.AutoPoses.testPose3,AutoConstants.AutoPoses.testPose4,
                                AutoConstants.AutoPoses.testPose5)));
      break;
    }
    autocommand = command;
    return command;
  }

  private void resetDefaultCommand(){
    m_swerve.setDefaultCommand( // m_swerve will execute this command periodically
    m_swerve.applyRequest(() -> drive.withVelocityX(-m_xBoxDriver.getRawAxis(translationAxis) * TunerConstants.kMaxSpeed) // Drive forward with
                                                                                        // negative Y (forward)
        .withVelocityY(-m_xBoxDriver.getRawAxis(strafeAxis) * TunerConstants.kMaxSpeed) // Drive left with negative X (left)
        .withRotationalRate(-m_xBoxDriver.getRawAxis(rotationAxis) * TunerConstants.kMaxSpeed) // Drive counterclockwise with negative X (left)
    ));
  }

}


  
  