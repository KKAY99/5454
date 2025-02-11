package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.events.TriggerEvent;
import com.revrobotics.spark.ClosedLoopSlot;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.*;
import frc.robot.utilities.AutoPlanner;
import frc.robot.utilities.JacksonsCoolPanel;
import frc.robot.utilities.Limelight;
import frc.robot.utilities.LimelightManager;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;
import frc.robot.commands.*;
import com.ctre.phoenix6.Utils;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import com.ctre.phoenix6.swerve.SwerveRequest;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.ButtonBindings;
import frc.robot.Constants.CoolPanelConstants;
import frc.robot.Constants.DunkinDonutConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.InputControllers;
import frc.robot.Constants.ElevatorConstants.ElevatorScoreLevel;

public class RobotContainer {
  private final Field2d m_Field2d = new Field2d();
  private final int translationAxis = XboxController.Axis.kLeftY.value;
  private final int strafeAxis = XboxController.Axis.kLeftX.value;
  private final int rotationAxis = XboxController.Axis.kRightX.value;
  
  private final int rightTriggerAxis = XboxController.Axis.kRightTrigger.value;
  private final int leftTriggerAxis = XboxController.Axis.kLeftTrigger.value;

  private final CommandXboxController m_xBoxDriver = new CommandXboxController(InputControllers.kXboxDrive);
  private XboxController m_xBoxOperator = new XboxController(InputControllers.kXboxOperator);
  private Joystick m_CustomController = new Joystick(InputControllers.kCustomController);

  //DunkinSubsystem
  private DunkinDonutSubsystem m_dunkinDonut = new DunkinDonutSubsystem(DunkinDonutConstants.coralCanID,DunkinDonutConstants.algaeCanID,
                                                                        DunkinDonutConstants.rotateCanID,DunkinDonutConstants.canCoderID);
  
  //ElevatorSubsystem
  private ElevatorSubsystem m_elevator = new ElevatorSubsystem(ElevatorConstants.elevatorCanID,ElevatorConstants.canAndColorID);

  public final CommandSwerveDrivetrain m_swerve = TunerConstants.createDrivetrain();
  public final Limelight m_OdomLimelight=new Limelight(Constants.LimeLightValues.limelightBackOdomHeight,Constants.LimeLightValues.limelightBackOdomAngle,
                                                0,Constants.LimeLightValues.backOdomLimelightName);
  /*public final Limelight m_OdomFwdLimelight=new Limelight(Constants.LimeLightValues.limelightFrontOdomHeight,Constants.LimeLightValues.limelightFrontOdomAngle,
                                                0,Constants.LimeLightValues.frontOdomLimelightName);*/

  //public final LimelightManager m_LimelightManager=new LimelightManager(m_OdomLimelight,m_OdomFwdLimelight);

  public final JacksonsCoolPanel m_JacksonsCoolPanel=new JacksonsCoolPanel(CoolPanelConstants.greenPWM,CoolPanelConstants.redPWM);

  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
          .withDeadband(TunerConstants.kMaxSpeed * 0.1).withRotationalDeadband(TunerConstants.kMaxAngularSpeed*0.1);
  
  private final SendableChooser<Command> m_autoChooser;

  public double m_P;
  public double m_I;
  public double m_D;

  public boolean hasHomed=false;

  public ElevatorScoreLevel m_currentScoreLevel=ElevatorScoreLevel.L1;

  public RobotContainer(){
    SmartDashboard.putData("field", m_Field2d); //used for elastic
    configureNamedCommands();
    m_autoChooser = AutoBuilder.buildAutoChooser();
    // Configure the button bindings
    createAutonomousCommandList(); 
    configureButtonBindings();
    //Create Auto Commands
    resetDefaultCommand();
  }

  public void configureNamedCommands() {
    /*NamedCommands.registerCommand("PipelineCenterApriltag",new PipelineSwapCommand(m_OdomLimelight,Constants.LimeLightValues.centerApriltagPipeline));
    NamedCommands.registerCommand("PipelineLeftApriltag",new PipelineSwapCommand(m_OdomLimelight,Constants.LimeLightValues.leftApriltagPipeline));
    NamedCommands.registerCommand("PipelineRightApriltag",new PipelineSwapCommand(m_OdomLimelight,Constants.LimeLightValues.rightApriltagPipeline));
    NamedCommands.registerCommand("ApriltagLineUp21",new ApriltagLineupCommand(m_swerve,m_OdomLimelight,00,21,true));
    NamedCommands.registerCommand("ApriltagLineUp19",new ApriltagLineupCommand(m_swerve,m_OdomLimelight,00,19,true));*/
  }

  private void configureButtonBindings(){
    /*PipelineSwapCommand piplineSwap0=new PipelineSwapCommand(m_OdomLimelight,Constants.LimeLightValues.centerApriltagPipeline);
    m_xBoxDriver.b().onTrue(piplineSwap0);

    PipelineSwapCommand piplineSwap1=new PipelineSwapCommand(m_OdomLimelight,Constants.LimeLightValues.leftApriltagPipeline);
    m_xBoxDriver.leftBumper().onTrue(piplineSwap1);

    PipelineSwapCommand piplineSwap2=new PipelineSwapCommand(m_OdomLimelight,Constants.LimeLightValues.rightApriltagPipeline);
    m_xBoxDriver.rightBumper().onTrue(piplineSwap2);*/

    //DunkinDonutCommands
    DunkinDonutRotateCommand DunkinRotateCommand = new DunkinDonutRotateCommand(m_dunkinDonut, () -> m_xBoxOperator.getRightX()*0.5);
    Trigger operatorRightXJoystick = new Trigger(() -> Math.abs(m_xBoxOperator.getRightX())>Constants.ButtonBindings.joystickDeadband);
    operatorRightXJoystick.whileTrue(DunkinRotateCommand);

    DunkinDonutCoralCommand DunkinCoralCommand = new DunkinDonutCoralCommand(m_dunkinDonut, -0.5);
    JoystickButton operatorDunkinCoralButton = new JoystickButton(m_xBoxOperator,Constants.ButtonBindings.dunkinCoralButton);
    operatorDunkinCoralButton.whileTrue(DunkinCoralCommand);

    DunkinDonutCoralCommand DunkinCoralCommandIn = new DunkinDonutCoralCommand(m_dunkinDonut, 0.5);
    JoystickButton operatorDunkinCoralButtonIn = new JoystickButton(m_xBoxOperator,3);
    operatorDunkinCoralButtonIn.whileTrue(DunkinCoralCommandIn);

    DunkinDonutAlgeaCommand DunkinAlgeaShootCommand = new DunkinDonutAlgeaCommand(m_dunkinDonut, -1,false); 
    JoystickButton operatorDunkinAlgeaShootButton = new JoystickButton(m_xBoxOperator, Constants.ButtonBindings.dunkinAlgeaShootButton);
    operatorDunkinAlgeaShootButton.whileTrue(DunkinAlgeaShootCommand);

    DunkinDonutAlgeaCommand DunkinAlgeaPullCommand = new DunkinDonutAlgeaCommand(m_dunkinDonut, 1,false); 
    JoystickButton operatorDunkinAlgeaPullButton = new JoystickButton(m_xBoxOperator, Constants.ButtonBindings.dunkinAlgeaPullButton);
    operatorDunkinAlgeaPullButton.whileTrue(DunkinAlgeaPullCommand);
    
    //ElevatorCommands
    ElevatorCommand ElevatorCommand = new ElevatorCommand(m_elevator, () -> m_xBoxOperator.getLeftY()*0.5);
    Trigger operatorLeftYJoystick = new Trigger(() -> Math.abs(m_xBoxOperator.getLeftY())>Constants.ButtonBindings.joystickDeadband);
    operatorLeftYJoystick.whileTrue(ElevatorCommand);

    //Sequential
    SequentialCommandGroup retract=new SequentialCommandGroup(new ToggleLocalDunkinPID(m_dunkinDonut,m_elevator,()->ElevatorScoreLevel.RETRACT,true),
                                                              new ABSRotateAtPos(m_dunkinDonut,()->ElevatorScoreLevel.RETRACT),
                                                              new ElevatorPosCommand(m_elevator,()->ElevatorScoreLevel.RETRACT,ClosedLoopSlot.kSlot1),
                                                              new ElevatorAndRotateAtPos(m_elevator,m_dunkinDonut,()->ElevatorScoreLevel.RETRACT),
                                                              new ToggleLocalDunkinPID(m_dunkinDonut,m_elevator,()->ElevatorScoreLevel.RETRACT));
    JoystickButton operatorRetractButton = new JoystickButton(m_xBoxOperator,Constants.ButtonBindings.retractButton);
    operatorRetractButton.onTrue(retract);

    SequentialCommandGroup seqScore=new SequentialCommandGroup(//new DunkinDonutAlgeaCommand(m_dunkinDonut,1,true),
                                                              new ParallelCommandGroup(new ElevatorPosCommand(m_elevator,()->m_currentScoreLevel),
                                                              new ToggleLocalDunkinPID(m_dunkinDonut,m_elevator,()->m_currentScoreLevel,true)),
                                                              new ElevatorAndRotateAtPos(m_elevator,m_dunkinDonut,()->m_currentScoreLevel),
                                                              new WaitCommand(0.75),
                                                              new DunkinDonutCoralCommand(m_dunkinDonut,0.5,1));
                                                              //new DunkinDonutAlgeaCommand(m_dunkinDonut,0,true));              
    JoystickButton operatorSeqScoreButton = new JoystickButton(m_xBoxOperator,Constants.ButtonBindings.elevatorScoreLevelButton);
    operatorSeqScoreButton.onTrue(seqScore);

    //Lineup
   /*PipelineSwapCommand pipelineSwapRight=new PipelineSwapCommand(m_OdomLimelight,Constants.LimeLightValues.rightApriltagPipeline);
    JoystickButton pipelineRightButton=new JoystickButton(m_xBoxOperator,ButtonBindings.pipelineSwap);
    pipelineRightButton.onTrue(pipelineSwapRight);*/

    /*ApriltagLineupCommand lineupApriltag=new ApriltagLineupCommand(m_swerve,m_OdomLimelight,29,20,false);
    Trigger lineUpTrigger=new Trigger(()->(m_xBoxOperator.getLeftTriggerAxis()>ButtonBindings.joystickDeadband));
    lineUpTrigger.whileTrue(lineupApriltag);*/

   OdomLineupCommand odomLineup=new OdomLineupCommand(m_OdomLimelight, m_swerve);
   m_xBoxDriver.leftBumper().onTrue(odomLineup);
  }

  public void setScoreLevelPOV(Supplier<Integer> pov){
    switch(pov.get()){
      case 0:
      m_currentScoreLevel=ElevatorScoreLevel.L1;
      break;
      case 90:
      m_currentScoreLevel=ElevatorScoreLevel.L2;
      break;
      case 180:
      m_currentScoreLevel=ElevatorScoreLevel.L3;
      break;
      case 270:
      m_currentScoreLevel=ElevatorScoreLevel.L4;
      break;
    }
  }
      
  private void refreshSmartDashboard(){  
    SmartDashboard.putNumber("Odom Limelight Distance", m_OdomLimelight.getDistance());
    SmartDashboard.putNumber("Odom Limelight X", m_OdomLimelight.getX());
    /*SmartDashboard.putNumber("Odom Limelight Distance", m_OdomLimelight.getDistance());
    SmartDashboard.putNumber("Odom Limelight X", m_OdomLimelight.getX());*/
    
    SmartDashboard.putNumber("Match Time", DriverStation.getMatchTime());
    SmartDashboard.putNumber("Votaage",RobotController.getBatteryVoltage());
    SmartDashboard.putNumber("Elevator Relative",m_elevator.getRelativePos());
    SmartDashboard.putNumber("Dunkin Rotate Relative",m_dunkinDonut.get_rotatemotorpos());
    SmartDashboard.putNumber("Dunkin Rotate ABS",m_dunkinDonut.getAbsoluteEncoderPos());
    SmartDashboard.putString("Current Score Level",m_currentScoreLevel.toString());
  }
  
  private void createAutonomousCommandList(){
    try{
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

  public BooleanSupplier checkCan(){
    return (() ->(m_swerve.checkCANConnections()&&m_dunkinDonut.checkCANConnections()&&m_elevator.checkCANConnections()));
  }

  public void DisabledInit(){
  }
   
  public void DisabledPeriodic(){ 
  }
  
  public void AutoPeriodic(){
    /*if(m_OdomLimelight.isAnyTargetAvailable()){
      m_OdomLimelight.SetRobotOrientation(m_swerve.getPigeon2().getYaw().getValueAsDouble(),
                                          m_swerve.getPigeon2().getAngularVelocityZWorld().getValueAsDouble());
      Pose2d currentPose=m_OdomLimelight.GetPoseViaMegatag2();
      m_OdomLimelight.TrimPoseArray(3);
      System.out.println("confidence: " + m_OdomLimelight.getConfidence(3,currentPose));
      if(m_OdomLimelight.getConfidence(3,currentPose)){
        m_swerve.addVisionMeasurement(currentPose,Utils.getCurrentTimeSeconds());
      }
    }

    if(m_OdomFwdLimelight.isAnyTargetAvailable()){
      m_OdomFwdLimelight.SetRobotOrientation(m_swerve.getPigeon2().getYaw().getValueAsDouble(),
                                          m_swerve.getPigeon2().getAngularVelocityZWorld().getValueAsDouble());
    }*/
  }

  public void AutonMode(){
    m_swerve.setVisionMeasurementStdDevs(VecBuilder.fill(.7,.7,9999999));
  }

  public void TeleopMode(){
    homeRobot();
    //m_OdomLimelight.resetLimelightIDFilter();
    m_swerve.setVisionMeasurementStdDevs(VecBuilder.fill(.7,.7,9999999));
    m_OdomLimelight.setLimelightIDFilter(21);
  }

  public void TeleopPeriodic(){
    refreshSmartDashboard();
    GetPIDValues();

    if(m_OdomLimelight.isAnyTargetAvailable()){
      m_OdomLimelight.SetRobotOrientation(m_swerve.getPigeon2().getYaw().getValueAsDouble(),
                                          m_swerve.getPigeon2().getAngularVelocityZWorld().getValueAsDouble());
    }
    /*if(m_OdomFwdLimelight.isAnyTargetAvailable()){
      m_OdomFwdLimelight.SetRobotOrientation(m_swerve.getPigeon2().getYaw().getValueAsDouble(),
                                          m_swerve.getPigeon2().getAngularVelocityZWorld().getValueAsDouble());*/
    }

  public void AllPeriodic(){
    setScoreLevelPOV(()->m_xBoxOperator.getPOV());
    m_JacksonsCoolPanel.isAllCanAvailable(checkCan());
  }

  public void homeRobot(){
    if(!hasHomed){
      hasHomed = true;
      //CommandScheduler.getInstance().schedule(new DunkinDonutHomeCommand(m_dunkinDonut));
      CommandScheduler.getInstance().schedule(new ElevatorHomeCommand(m_elevator));
    }
  }

  public Command getAutonomousCommand(){
    //Command command=m_autoChooser.getSelected();
    Command command = m_autoChooser.getSelected();
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


  
  