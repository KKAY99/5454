package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
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
  public final Limelight m_OdomLimelight=new Limelight(Constants.LimeLightValues.limelightOdomHeight,Constants.LimeLightValues.limelightOdomAngle,
                                                0,Constants.LimeLightValues.odomLimelightName);

  public final Limelight m_NeuralLimelight=new Limelight(Constants.LimeLightValues.limelightNeuralHeight,Constants.LimeLightValues.limelightNeuralAngle,
                                                0,Constants.LimeLightValues.neuralLimelightName);

  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
          .withDeadband(TunerConstants.kMaxSpeed * 0.1).withRotationalDeadband(TunerConstants.kMaxAngularSpeed*0.1);

  public double m_P;
  public double m_I;
  public double m_D;

  public RobotContainer(){
    configureNamedCommands();
    m_autoChooser = AutoBuilder.buildAutoChooser();
    // Configure the button bindings
    createAutonomousCommandList(); 
    configureButtonBindings();
    //Create Auto Commands
    resetDefaultCommand();
  }

  public void configureNamedCommands() {
    NamedCommands.registerCommand("PipelineCenterApriltag",new PipelineSwapCommand(m_OdomLimelight,Constants.LimeLightValues.centerApriltagPipeline));
    NamedCommands.registerCommand("PipelineLeftApriltag",new PipelineSwapCommand(m_OdomLimelight,Constants.LimeLightValues.leftApriltagPipeline));
    NamedCommands.registerCommand("PipelineRightApriltag",new PipelineSwapCommand(m_OdomLimelight,Constants.LimeLightValues.rightApriltagPipeline));
    NamedCommands.registerCommand("ApriltagLineUp",new LimelightLineupCommand(m_swerve,m_OdomLimelight,30,21));
    NamedCommands.registerCommand("ObjectLineUp",new LimelightLineupCommand(m_swerve,m_NeuralLimelight,40));
  }

  private void configureButtonBindings(){
    LimelightLineupCommand lineupApriltag=new LimelightLineupCommand(m_swerve, m_OdomLimelight,30,21);
    m_xBoxDriver.leftTrigger().whileTrue(lineupApriltag);

    LimelightLineupCommand lineupObject=new LimelightLineupCommand(m_swerve, m_NeuralLimelight,40);
    m_xBoxDriver.rightTrigger().whileTrue(lineupObject);

    PipelineSwapCommand piplineSwap0=new PipelineSwapCommand(m_OdomLimelight,Constants.LimeLightValues.centerApriltagPipeline);
    m_xBoxDriver.b().onTrue(piplineSwap0);

    PipelineSwapCommand piplineSwap1=new PipelineSwapCommand(m_OdomLimelight,Constants.LimeLightValues.leftApriltagPipeline);
    m_xBoxDriver.leftBumper().onTrue(piplineSwap1);

    PipelineSwapCommand piplineSwap2=new PipelineSwapCommand(m_OdomLimelight,Constants.LimeLightValues.rightApriltagPipeline);
    m_xBoxDriver.rightBumper().onTrue(piplineSwap2);
  }
      
  private void refreshSmartDashboard(){  
    SmartDashboard.putNumber("Odom Limelight Distance", m_OdomLimelight.getDistance());
    SmartDashboard.putNumber("Odom Limelight X", m_OdomLimelight.getX());
    SmartDashboard.putNumber("Neural Limelight Distance", m_NeuralLimelight.getDistance());
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
    if(m_OdomLimelight.isTargetAvailible()){
      Pose2d poseFromHelp=m_OdomLimelight.GetPoseViaHelper(m_swerve.getPigeon2().getYaw().getValueAsDouble());
      m_swerve.addVisionMeasurement(new Pose2d(poseFromHelp.getX(),poseFromHelp.getY(),new Rotation2d(0)),
                                    Utils.getCurrentTimeSeconds());
    }
  }

  public void AutonMode(){
    m_swerve.setVisionMeasurementStdDevs(VecBuilder.fill(.7,.7,9999999));
  }

  public void TeleopMode(){
    m_OdomLimelight.setIDFilter(21);
  }

  public void TeleopPeriodic(){
    refreshSmartDashboard();
    GetPIDValues();

    //System.out.println("Distance to object: " + m_NeuralLimelight.getDistance());
    /*if(m_OdomLimelight.isTargetAvailible()){
      System.out.println("Pose Via AprilTagMG: "+m_OdomLimelight.GetPoseViaHelper(m_swerve.getPigeon2().getYaw().getValueAsDouble()));
      System.out.println("Pose Via AprilTagNT: "+m_OdomLimelight.GetPoseViaApriltagNT());
      //m_swerve.addVisionMeasurement(m_OdomLimelight.GetPoseViaHelper(m_swerve.getPigeon2().getYaw().getValueAsDouble()),
      //                               Utils.getCurrentTimeSeconds());
     }*/
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


  
  