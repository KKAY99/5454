package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.events.TriggerEvent;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
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
import frc.robot.Constants.ButtonBindings;
import frc.robot.Constants.ClimbConstants;
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

  private final CommandXboxController m_xBoxDriver = new CommandXboxController(InputControllers.kXboxDrive);
  private XboxController m_xBoxOperator = new XboxController(InputControllers.kXboxOperator);

  //DunkinSubsystem
  private DunkinDonutSubsystem m_dunkinDonut = new DunkinDonutSubsystem(DunkinDonutConstants.coralCanID,DunkinDonutConstants.algaeCanID1,DunkinDonutConstants.algaeCanID2,
                                                                        DunkinDonutConstants.rotateCanID,DunkinDonutConstants.canCoderID);
  
  //ElevatorSubsystem
  private ElevatorSubsystem m_elevator = new ElevatorSubsystem(ElevatorConstants.elevatorCanID,ElevatorConstants.canAndColorID);

  //ClimbSubsystem
  private ClimbSubsystem m_climb=new ClimbSubsystem(ClimbConstants.climbCanID,ClimbConstants.encoderDIO);

  public final CommandSwerveDrivetrain m_swerve = TunerConstants.createDrivetrain();
  public final Limelight m_OdomLimelight=new Limelight(Constants.LimeLightValues.limelightBackOdomHeight,Constants.LimeLightValues.limelightBackOdomAngle,
                                                0,Constants.LimeLightValues.backOdomLimelightName);
  /*public final Limelight m_OdomFwdLimelight=new Limelight(Constants.LimeLightValues.limelightFrontOdomHeight,Constants.LimeLightValues.limelightFrontOdomAngle,
                                                0,Constants.LimeLightValues.frontOdomLimelightName);*/

  //public final LimelightManager m_LimelightManager=new LimelightManager(m_OdomLimelight,m_OdomFwdLimelight);

  public final JacksonsCoolPanel m_JacksonsCoolPanel=new JacksonsCoolPanel(CoolPanelConstants.greenPWM,CoolPanelConstants.redPWM);
  
  private final SendableChooser<Command> m_autoChooser;

  public ElevatorScoreLevel m_currentScoreLevel=ElevatorScoreLevel.L1;

  public double m_P;
  public double m_I;
  public double m_D;

  public boolean hasHomed=false;
  public boolean m_isRightLineup=false;

  public RobotContainer(){
    SmartDashboard.putData("field", m_Field2d); //used for elastic
    configureNamedCommands();
    m_autoChooser=AutoBuilder.buildAutoChooser();
    createAutonomousCommandList(); 
    configureButtonBindings();
    resetDefaultCommand();
  }

  public void configureNamedCommands() {
    NamedCommands.registerCommand("AutoScoreLeft",new AutoScoreCommand(m_elevator,m_dunkinDonut,m_OdomLimelight,()->m_currentScoreLevel,false));
    NamedCommands.registerCommand("AutoScoreRight",new AutoScoreCommand(m_elevator,m_dunkinDonut,m_OdomLimelight,()->m_currentScoreLevel,true));
  }

  private void configureButtonBindings(){
    //QOL Drive
    ResetGyroCommand resetGyroCommand=new ResetGyroCommand(m_swerve);
    m_xBoxDriver.start().onTrue(resetGyroCommand);

    GasPedalCommand gasPedalCommand=new GasPedalCommand(m_swerve,()->m_xBoxDriver.getRightTriggerAxis());
    m_xBoxDriver.rightTrigger().whileTrue(gasPedalCommand);

    //Climb
    ClimbRotateCommand rotateFwdCommand=new ClimbRotateCommand(m_climb,1);
    m_xBoxDriver.a().whileTrue(rotateFwdCommand);

    ClimbRotateCommand rotateBwdCommand=new ClimbRotateCommand(m_climb,-1);
    m_xBoxDriver.b().whileTrue(rotateBwdCommand);
    
    /*ToggleClimbPID testPID1=new ToggleClimbPID(m_climb,ClimbConstants.climbPos1);
    m_xBoxDriver.leftBumper().onTrue(testPID1);

    ToggleClimbPID testPID2=new ToggleClimbPID(m_climb,ClimbConstants.climbPos2);
    m_xBoxDriver.rightBumper().onTrue(testPID2);*/

    //DunkinDonutCommands
    /*DunkinDonutRotateCommand DunkinRotateCommand = new DunkinDonutRotateCommand(m_dunkinDonut, () -> m_xBoxOperator.getRightX()*0.5);
    Trigger operatorRightXJoystick = new Trigger(() -> Math.abs(m_xBoxOperator.getRightX())>Constants.ButtonBindings.joystickDeadband);
    operatorRightXJoystick.whileTrue(DunkinRotateCommand);*/

    DunkinDonutCoralCommand DunkinCoralCommand = new DunkinDonutCoralCommand(m_dunkinDonut, -0.75);
    JoystickButton operatorDunkinCoralButton = new JoystickButton(m_xBoxOperator,Constants.ButtonBindings.dunkinCoralOutakeButton);
    operatorDunkinCoralButton.whileTrue(DunkinCoralCommand);

    DunkinDonutCoralCommand DunkinCoralCommandIn = new DunkinDonutCoralCommand(m_dunkinDonut, 0.20);
    JoystickButton operatorDunkinCoralButtonIn = new JoystickButton(m_xBoxOperator,Constants.ButtonBindings.dunkinCoralIntakeButton);
    operatorDunkinCoralButtonIn.whileTrue(DunkinCoralCommandIn);
    
    /*DunkinDonutAlgeaCommand DunkinAlgeaShootCommand = new DunkinDonutAlgeaCommand(m_dunkinDonut, -1,false); 
    JoystickButton operatorDunkinAlgeaShootButton = new JoystickButton(m_xBoxOperator,2);
    operatorDunkinAlgeaShootButton.whileTrue(DunkinAlgeaShootCommand);

    DunkinDonutAlgeaCommand DunkinAlgeaPullCommand = new DunkinDonutAlgeaCommand(m_dunkinDonut, 0.5,false); 
    JoystickButton operatorDunkinAlgeaPullButton = new JoystickButton(m_xBoxOperator,3);
    operatorDunkinAlgeaPullButton.whileTrue(DunkinAlgeaPullCommand);*/
    
    //ElevatorCommands
    ElevatorCommand ElevatorCommand = new ElevatorCommand(m_elevator, () -> m_xBoxOperator.getLeftY()*0.5);
    Trigger operatorLeftYJoystick = new Trigger(()->Math.abs(m_xBoxOperator.getLeftY())>Constants.ButtonBindings.joystickDeadband);
    operatorLeftYJoystick.whileTrue(ElevatorCommand);

    AutoScoreCommand seqScoreCommandManual=new AutoScoreCommand(m_elevator,m_dunkinDonut,m_OdomLimelight,()->m_currentScoreLevel,false);
    JoystickButton operatorSeqScoreManualButton=new JoystickButton(m_xBoxOperator,Constants.ButtonBindings.elevatorScoreLevelButton);
    operatorSeqScoreManualButton.onTrue(seqScoreCommandManual);
 
    /*SequentialCommandGroup seqScoreCommandAuto = new AutoScoreCommand(m_swerve,m_elevator,m_dunkinDonut,m_OdomLimelight,()->m_currentScoreLevel,m_isRightLineup);
    JoystickButton operatorSeqScoreAuto = new JoystickButton(m_xBoxOperator, ButtonBindings.lineUpButton);
    operatorSeqScoreAuto.onTrue(seqScoreCommandAuto);*/

    //Lineup
    OdomLineupCommand odomLineupCommand=new OdomLineupCommand(m_OdomLimelight,m_swerve,m_isRightLineup);
    JoystickButton odomLineupButton=new JoystickButton(m_xBoxOperator,ButtonBindings.lineUpButton);
    odomLineupButton.onTrue(odomLineupCommand);
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

  public void setLineupSide(Supplier<Boolean> x,Supplier<Boolean> a){
    if(x.get()){
      m_isRightLineup=true;
    }

    if(a.get()){
      m_isRightLineup=false;
    }
  } 
      
  private void refreshSmartDashboard(){  
    SmartDashboard.putNumber("Elevator Relative",m_elevator.getRelativePos());
    SmartDashboard.putNumber("Dunkin Rotate Relative",m_dunkinDonut.get_rotatemotorpos());
    SmartDashboard.putNumber("Dunkin Rotate ABS",m_dunkinDonut.getAbsoluteEncoderPos());
    SmartDashboard.putString("Current Score Level",m_currentScoreLevel.toString());
    SmartDashboard.putNumber("Climb ABS Pos",m_climb.getAbsoluteEncoderPos());
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
    return (() ->(m_swerve.checkCANConnections()&&m_dunkinDonut.checkCANConnections()&&m_elevator.checkCANConnections()&&m_climb.checkCANConnections()));
  }

  public void DisabledInit(){}
   
  public void DisabledPeriodic(){}
  
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
    m_swerve.setVisionMeasurementStdDevs(VecBuilder.fill(.7,.7,9999999));
  }

  public void TeleopPeriodic(){
    refreshSmartDashboard();
    GetPIDValues();

    if(m_OdomLimelight.isAnyTargetAvailable()){
      m_OdomLimelight.SetRobotOrientation(m_swerve.getPigeon2().getYaw().getValueAsDouble(),
                                          m_swerve.getPigeon2().getAngularVelocityZWorld().getValueAsDouble());

      Pose2d currentPose=m_OdomLimelight.GetPoseViaMegatag2();
      double currentTimeStamp=Timer.getFPGATimestamp();
      m_OdomLimelight.TrimPoseArray(3);

      if(m_OdomLimelight.getDerivationConfidence(m_swerve,3,currentPose,currentTimeStamp)){
        m_swerve.addVisionMeasurement(currentPose,Utils.getCurrentTimeSeconds());
      }
    }
  }

  public void AllPeriodic(){
    m_Field2d.setRobotPose(m_swerve.getPose2d()); //elastic
    SmartDashboard.putNumber("Match Time", DriverStation.getMatchTime()); //elastic
    SmartDashboard.putNumber("Voltage",RobotController.getBatteryVoltage()); //elastic
    m_JacksonsCoolPanel.isAllCanAvailable(checkCan());
    setScoreLevelPOV(()->m_xBoxOperator.getPOV());
    setLineupSide(()->m_xBoxOperator.getXButtonPressed(),()->m_xBoxOperator.getAButtonPressed());
  }

  public void homeRobot(){
    if(!hasHomed){
      hasHomed = true;
      CommandScheduler.getInstance().schedule(new ElevatorHomeCommand(m_elevator));
    }
  }

  public Command getAutonomousCommand(){
    Command command=m_autoChooser.getSelected();
    return command;
  }
 
  private void resetDefaultCommand(){
    m_swerve.setDefaultCommand(m_swerve.applyRequestDrive(m_xBoxDriver,translationAxis,strafeAxis,rotationAxis));
  }
}


  
  