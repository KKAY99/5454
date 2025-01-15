package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.*;
import frc.robot.utilities.AutoPlanner;
import frc.robot.utilities.Limelight;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.*;

import com.ctre.phoenix6.Utils;
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

  private final SendableChooser<Command> m_autoChooser;

  //private SendableChooser<String> m_autoChooser = new SendableChooser<>(); 
  public final CommandSwerveDrivetrain m_swerve = TunerConstants.createDrivetrain();
  public final Limelight m_Limelight=new Limelight(Constants.LimeLightValues.targetHeight,Constants.LimeLightValues.limelightStaticHeight,Constants.LimeLightValues.limelightStaticAngle);


  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
          .withDeadband(TunerConstants.kMaxSpeed * 0.1).withRotationalDeadband(TunerConstants.kMaxAngularSpeed*0.1);

  public double m_P;
  public double m_I;
  public double m_D;

  public RobotContainer(){
    m_autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser",m_autoChooser);
    // Configure the button bindings
    configureButtonBindings();
    //createAutonomousCommandList(); 
    //Create Auto Commands
    resetDefaultCommand();
    m_Limelight.setPipeline(1);
  }


  public void configureNamedCommands() {


    }


  private void configureButtonBindings(){
    ApriltagLineupCommand lineup=new ApriltagLineupCommand(m_swerve, m_Limelight);
    m_xBoxDriver.a().whileTrue(lineup);
  }
      
  private void refreshSmartDashboard(){  
  }
  
  /*private boolean checkCANConnections(){
   return m_swerve.checkCANConnections();
  }
  */
  private void createAutonomousCommandList(){
    try{
     /* m_autoChooser.setDefaultOption(AutoConstants.autoMode1,AutoConstants.autoMode1);
      m_autoChooser.addOption(AutoConstants.autoMode2,AutoConstants.autoMode2);
      m_autoChooser.addOption(AutoConstants.autoMode3,AutoConstants.autoMode3);*/

      SmartDashboard.putNumber("P",m_P);
      SmartDashboard.putNumber("I",m_I);
      SmartDashboard.putNumber("D",m_D);

      SmartDashboard.putData("Auto Chooser",m_autoChooser);

    }catch(Exception e){
      System.out.println("Create Autos Failed, Exception: " + e.getMessage());
    }
  }

  public void GetPIDValues(){
    m_P=SmartDashboard.getNumber("P",0);
    m_I=SmartDashboard.getNumber("I",0);
    m_D=SmartDashboard.getNumber("D",0);
  }

  public void DisabledInit(){
  }
   
  public void DisabledPeriodic(){ 
  }
  
  public void AutoPeriodic(){
    if(m_Limelight.isTargetAvailible()){
      System.out.println("Pose Via AprilTag: "+m_Limelight.GetPoseViaApriltag());
      System.out.println("robot pos: " + m_swerve.getPose2d());
      m_swerve.addVisionMeasurement(m_Limelight.GetPoseViaApriltag(),Utils.getCurrentTimeSeconds());
    }
  }

  public void AutonMode(){
  }

  public void TeleopMode(){
  }

  public void TeleopPeriodic(){
    refreshSmartDashboard();
    GetPIDValues();

    if(m_Limelight.isTargetAvailible()){
      //System.out.println("Pose Via AprilTag: "+m_Limelight.GetPoseViaApriltag());
      //m_swerve.addVisionMeasurement(m_Limelight.GetPoseViaApriltag(),Timer.getFPGATimestamp());
    }else{
      //System.out.println("No Target");
    }
  }

  public void AllPeriodic(){
  }

  public Command getAutonomousCommand(){
    //Command command=m_autoChooser.getSelected();
    Command command = m_autoChooser.getSelected();
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
      m_swerve.resetPose(AutoConstants.AutoPoses.testStartPos);
      command=new SequentialCommandGroup(
        m_swerve.createPathCommand(
        autoPlan.CreateAutoPath(0,0,m_swerve.getPose2d(),AutoConstants.AutoPoses.testPose2)),
        m_swerve.createPathCommand(
        autoPlan.CreateAutoPath(0,0,AutoConstants.AutoPoses.testPose2,AutoConstants.AutoPoses.testPose3)));

      break;
      case AutoConstants.autoMode3:
      m_swerve.resetPose(AutoConstants.AutoPoses.centerStart);
      command=autoPlan.CreatePathfindingPath(0,AutoConstants.AutoPoses.pathFindingTestPose1);
      break;
    }

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


  
  