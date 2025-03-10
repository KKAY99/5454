package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.events.TriggerEvent;
import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
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
import java.util.function.UnaryOperator;
import frc.robot.commands.*;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import com.ctre.phoenix6.swerve.SwerveRequest;
import frc.robot.Constants.ButtonBindings;
import frc.robot.Constants.ClimbConstants;
import frc.robot.Constants.CoolPanelConstants;
import frc.robot.Constants.CoralConstants;
import frc.robot.Constants.DunkinDonutConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.InputControllers;
import frc.robot.Constants.LineupConstants;
import frc.robot.Constants.ElevatorConstants.ElevatorScoreLevel;
import org.littletonrobotics.junction.Logger;

public class RobotContainer {
  private final Field2d m_Field2d = new Field2d();
  
  private final int translationAxis = XboxController.Axis.kLeftY.value;
  private final int strafeAxis = XboxController.Axis.kLeftX.value;
  private final int rotationAxis = XboxController.Axis.kRightX.value;

  private final CommandXboxController m_xBoxDriver = new CommandXboxController(InputControllers.kXboxDrive);
  private XboxController m_xBoxOperator = new XboxController(InputControllers.kXboxOperator);

  //DunkinSubsystem
  private DunkinDonutSubsystem m_dunkinDonut = new DunkinDonutSubsystem(DunkinDonutConstants.coralCanID,DunkinDonutConstants.algaeCanID1,DunkinDonutConstants.algaeCanID2,
                                                                        DunkinDonutConstants.rotateCanID,DunkinDonutConstants.canCoderID,DunkinDonutConstants.limitSwitchDIO,DunkinDonutConstants.coralIndexerID,DunkinDonutConstants.indexerLimitSwitchDIO);
  
  //ElevatorSubsystem
  private ElevatorSubsystem m_elevator = new ElevatorSubsystem(ElevatorConstants.elevatorCanID,ElevatorConstants.canAndColorID);

  //ClimbSubsystem
  private ClimbSubsystem m_climb=new ClimbSubsystem(ClimbConstants.climbCanID1,ClimbConstants.climbCanID2,ClimbConstants.encoderDIO,ClimbConstants.ServoPMW);

  public final CommandSwerveDrivetrain m_swerve = TunerConstants.createDrivetrain();
  public final Limelight m_leftLimelight=new Limelight(Constants.LimeLightValues.leftLimelightHeight,Constants.LimeLightValues.leftLimelightAngle,
                                                0,Constants.LimeLightValues.leftLimelightName);
  public final Limelight m_rightLimelight=new Limelight(Constants.LimeLightValues.rightLimelightHeight,Constants.LimeLightValues.rightLimelightAngle,
                                                0,Constants.LimeLightValues.rightLimelightName);

  //public final LimelightManager m_LimelightManager=new LimelightManager(m_OdomLimelight,m_OdomFwdLimelight);

  public final JacksonsCoolPanel m_JacksonsCoolPanel=new JacksonsCoolPanel(CoolPanelConstants.greenPWM,CoolPanelConstants.redPWM);
  
  private final SendableChooser<Command> m_autoChooser;

  public ElevatorScoreLevel m_currentScoreLevel=ElevatorScoreLevel.L1;

  public double m_P;
  public double m_I;
  public double m_D;
  public double m_elevatorPos;

  public boolean m_ElevatorLevel1;
  public boolean m_ElevatorLevel2;
  public boolean m_ElevatorLevel3;
  public boolean m_ElevatorLevel4;


  public boolean hasHomed=false;
  public boolean m_isRightLineup=false;
  public boolean m_doAlgae=false;

  public RobotContainer(){
    SmartDashboard.putData("field", m_Field2d);
    configureNamedCommands();
    m_autoChooser=AutoBuilder.buildAutoChooser();
    createAutonomousCommandList(); 
    configureButtonBindings();
    resetDefaultCommand();
  }

  public void configureNamedCommands() {
    NamedCommands.registerCommand("AutoScoreLeft",new AutoScoreCommand(m_swerve,m_elevator,m_dunkinDonut,()->m_currentScoreLevel,m_leftLimelight,m_rightLimelight,()->false,()->false));
    NamedCommands.registerCommand("AutoScoreRight",new AutoScoreCommand(m_swerve,m_elevator,m_dunkinDonut,()->m_currentScoreLevel,m_leftLimelight,m_rightLimelight,()->true,()->false));
  }

  private void configureButtonBindings(){
    //QOL Drive
    /*ResetGyroCommand resetGyroCommand=new ResetGyroCommand(m_swerve);
    m_xBoxDriver.start().onTrue(resetGyroCommand);*/

    GasPedalCommand gasPedalCommand=new GasPedalCommand(m_swerve,()->m_xBoxDriver.getRightTriggerAxis());
    m_xBoxDriver.rightTrigger().whileTrue(gasPedalCommand);

    //Climb
    ClimbRotateCommand rotateFwdCommand=new ClimbRotateCommand(m_climb,ClimbConstants.climbForwardSpeed); //0.5 was to strong and bent the shaft...so we decided to up the power XD
    m_xBoxDriver.a().whileTrue(rotateFwdCommand);

    ClimbRotateCommand rotateBwdCommand=new ClimbRotateCommand(m_climb,ClimbConstants.climbBackSpeed);
    m_xBoxDriver.b().whileTrue(rotateBwdCommand);
    
    /*ToggleClimbPID testPID1=new ToggleClimbPID(m_climb,ClimbConstants.climbPos1);
    m_xBoxDriver.leftBumper().onTrue(testPID1);

    ToggleClimbPID testPID2=new ToggleClimbPID(m_climb,ClimbConstants.climbPos2);
    m_xBoxDriver.rightBumper().onTrue(testPID2);*/

    //DunkinDonutCommands
   /*DunkinDonutRotateCommand DunkinRotateCommand=new DunkinDonutRotateCommand(m_dunkinDonut,()->m_xBoxOperator.getRightX()*0.5);
    Trigger operatorRightXJoystick=new Trigger(()->Math.abs(m_xBoxOperator.getRightX())>Constants.ButtonBindings.joystickDeadband);
    operatorRightXJoystick.whileTrue(DunkinRotateCommand);

    ClawPIDScoreIntake clawProcessorScore=new ClawPIDScoreIntake(m_dunkinDonut,m_elevator,ElevatorConstants.processorScorePos,DunkinDonutConstants.processorScorePos,DunkinDonutConstants.processorScoreSpeed,
                                                                ElevatorConstants.elevatorLowLimit,DunkinDonutConstants.rotateHomePos);
    Trigger processorRightTrigger=new Trigger(()->Math.abs(m_xBoxOperator.getRightTriggerAxis())>ButtonBindings.joystickDeadband);
    processorRightTrigger.whileTrue(clawProcessorScore);

    ClawPIDScoreIntake clawLollipopIntake=new ClawPIDScoreIntake(m_dunkinDonut,m_elevator,ElevatorConstants.lollipopGrabPos,DunkinDonutConstants.lollipopGrabPos,DunkinDonutConstants.lollipopGrabSpeed,
                                                              ElevatorConstants.elevatorLowLimit,DunkinDonutConstants.algaeStowPos);
    Trigger lollipopIntakeLeftTrigger=new Trigger(()->Math.abs(m_xBoxOperator.getLeftTriggerAxis())>ButtonBindings.joystickDeadband);
    lollipopIntakeLeftTrigger.whileTrue(clawLollipopIntake);*/

    /*ClawPIDScoreIntake clawGroundIntake=new ClawPIDScoreIntake(m_dunkinDonut,m_elevator,ElevatorConstants.groundIntakePos,DunkinDonutConstants.groundIntakePos,DunkinDonutConstants.groundIntakeSpeed,
                                                              ElevatorConstants.elevatorLowLimit,DunkinDonutConstants.algaeStowPos);
    Trigger clawGroundIntakeLeftTrigger=new Trigger(()->Math.abs(m_xBoxOperator.getLeftTriggerAxis())>ButtonBindings.joystickDeadband);
    clawGroundIntakeLeftTrigger.whileTrue(clawGroundIntake);*/

    DunkinDonutCoralCommand DunkinCoralCommandIntake = new DunkinDonutCoralCommand(m_dunkinDonut, m_elevator, CoralConstants.coralIntakeSpeed, true, true, 0.75, 0.35);
    JoystickButton operatorDunkinCoralButtonIntake = new JoystickButton(m_xBoxOperator,Constants.ButtonBindings.dunkinCoralIntakeButton);
    operatorDunkinCoralButtonIntake.onTrue(DunkinCoralCommandIntake);

    DunkinDonutCoralCommand DunkinCoralCommand = new DunkinDonutCoralCommand(m_dunkinDonut, CoralConstants.coralOutakeSpeed,false, true,-0.25, -0.25);
    JoystickButton operatorDunkinCoralButton = new JoystickButton(m_xBoxOperator,Constants.ButtonBindings.dunkinCoralOutakeButton);
    operatorDunkinCoralButton.whileTrue(DunkinCoralCommand);
    
    //ElevatorCommands
    ElevatorCommand ElevatorCommand = new ElevatorCommand(m_elevator, () -> m_xBoxOperator.getLeftY()*0.5);
    Trigger operatorLeftYJoystick = new Trigger(()->Math.abs(m_xBoxOperator.getLeftY())>Constants.ButtonBindings.joystickDeadband);
    operatorLeftYJoystick.whileTrue(ElevatorCommand);

    AutoScoreCommand seqScoreCommandManual=new AutoScoreCommand(m_elevator,m_dunkinDonut,()->m_currentScoreLevel,()->false);
    JoystickButton operatorSeqScoreManualButton=new JoystickButton(m_xBoxOperator,Constants.ButtonBindings.elevatorScoreManualButton);
    operatorSeqScoreManualButton.onTrue(seqScoreCommandManual);
 
   /* SequentialCommandGroup seqScoreCommandAuto = new AutoScoreCommand(m_swerve,m_elevator,m_dunkinDonut,()->m_currentScoreLevel,m_leftLimelight,m_rightLimelight,()->m_isRightLineup);
    JoystickButton operatorSeqScoreAuto = new JoystickButton(m_xBoxOperator,Constants.ButtonBindings.elevatorScoreAutoButton);
    operatorSeqScoreAuto.onTrue(seqScoreCommandAuto);*/

    //Lineup
    ApriltagLineupCommand lineup=new ApriltagLineupCommand(m_swerve,m_leftLimelight,m_rightLimelight,()->m_isRightLineup);
    JoystickButton lineupButton=new JoystickButton(m_xBoxOperator,5);
    lineupButton.whileTrue(lineup);
  }

  public void setScoreLevelPOV(Supplier<Integer> pov){
    switch(pov.get()){
      case 90:
      m_currentScoreLevel=ElevatorScoreLevel.L1;
      m_ElevatorLevel1 = true;
      m_ElevatorLevel2 = false;
      m_ElevatorLevel3 = false;
      m_ElevatorLevel4 = false;
      break;
      case 180:
      m_currentScoreLevel=ElevatorScoreLevel.L2;
      m_ElevatorLevel1 = false;
      m_ElevatorLevel2 = true;
      m_ElevatorLevel3 = false;
      m_ElevatorLevel4 = false;
      break;
      case 270:
      m_currentScoreLevel=ElevatorScoreLevel.L3;
      m_ElevatorLevel1 = false;
      m_ElevatorLevel2 = false;
      m_ElevatorLevel3 = true;
      m_ElevatorLevel4 = false;
      break;
      case 0:
      m_currentScoreLevel=ElevatorScoreLevel.L4;
      m_ElevatorLevel1 = false;
      m_ElevatorLevel2 = false;
      m_ElevatorLevel3 = false;
      m_ElevatorLevel4 = true;
      break;
    }
  }

  public void setLineupSide(Supplier<Boolean> x,Supplier<Boolean> b){
    if(x.get()){
      m_isRightLineup=false;
    }

    if(b.get()){
      m_isRightLineup=true;
    }
  } 

  public void setDoesDoAlgae(Supplier<Boolean> leftBumperPress){
    m_doAlgae=false;//(m_doAlgae&&leftBumperPress.get())?false:true;
  } 
      
  private void refreshSmartDashboard(){  
    SmartDashboard.putNumber("Elevator Relative",m_elevator.getRelativePos());
    //SmartDashboard.putNumber("Dunkin Rotate Relative",m_dunkinDonut.get_rotatemotorpos());
    SmartDashboard.putBoolean("m_ElevatorLevel1", m_ElevatorLevel1);
    SmartDashboard.putBoolean("m_ElevatorLevel2", m_ElevatorLevel2);
    SmartDashboard.putBoolean("m_ElevatorLevel3", m_ElevatorLevel3);
    SmartDashboard.putBoolean("m_ElevatorLevel4", m_ElevatorLevel4);
    
    SmartDashboard.putBoolean("LEFT Lineup",m_isRightLineup==false);
    SmartDashboard.putBoolean("RIGHT Lineup",m_isRightLineup==true);
    SmartDashboard.putBoolean("Do Algea", m_doAlgae);
    SmartDashboard.putNumber("Dunkin Rotate ABS",m_dunkinDonut.getAbsoluteEncoderPos());
    SmartDashboard.putString("Current Score Level",m_currentScoreLevel.toString());
    SmartDashboard.putNumber("Climb ABS Pos",m_climb.getAbsoluteEncoderPos());
    SmartDashboard.putNumber("LEFT LIMELIGHT X",m_leftLimelight.getX());
    SmartDashboard.putNumber("RIGHT LIMELIGHT X",m_rightLimelight.getX());
  }
  
  private void createAutonomousCommandList(){
    try{
      SmartDashboard.putNumber("P",m_P);
      SmartDashboard.putNumber("I",m_I);
      SmartDashboard.putNumber("D",m_D);
      SmartDashboard.putNumber("Elevator pos",m_elevatorPos);

      SmartDashboard.putData("Auto Chooser",m_autoChooser);

    }catch(Exception e){
      System.out.println("Create Autos Failed, Exception: " + e.getMessage());
    }
  }

  public void GetElevatorPos(){
    m_elevatorPos=SmartDashboard.getNumber("Elevator pos", 0);
  }

  public void GetPIDValues(){
    m_P=SmartDashboard.getNumber("P",0);
    m_I=SmartDashboard.getNumber("I",0);
    m_D=SmartDashboard.getNumber("D",0);
  }

  public BooleanSupplier checkCan(){
    return (()->(m_swerve.checkCANConnections()&&m_dunkinDonut.checkCANConnections()&&m_elevator.checkCANConnections()&&m_climb.checkCANConnections()));
  }

  public void DisabledInit(){
    m_swerve.setVisionMeasurementStdDevs(VecBuilder.fill(5,5,9999999));
  }
   
  public void DisabledPeriodic(){
    resetGyroPoseDisabled();
  }
  
  public void AutoPeriodic(){
    /*if(m_OdomLimelight.isAnyTargetAvailable()){
      if(DriverStation.getAlliance().get()==Alliance.Blue){
        m_OdomLimelight.SetRobotOrientation(m_swerve.getPigeon2().getRotation2d().getDegrees(),
                                          m_swerve.getPigeon2().getAngularVelocityZWorld().getValueAsDouble());
      }else{
        m_OdomLimelight.SetRobotOrientation(m_swerve.getPigeon2().getRotation2d().getDegrees(),
                                          m_swerve.getPigeon2().getAngularVelocityZWorld().getValueAsDouble());
        /*m_OdomLimelight.SetRobotOrientation(m_swerve.getPigeon2().getRotation2d().getDegrees()-180,
                                          0-m_swerve.getPigeon2().getAngularVelocityZWorld().getValueAsDouble());
      }

      Pose2d currentPose=m_OdomLimelight.GetPoseViaMegatag2();
      double currentTimeStamp=Timer.getFPGATimestamp();
      m_OdomLimelight.TrimPoseArray(3);

      System.out.println(m_OdomLimelight.getDerivationConfidence(m_swerve,currentPose,currentTimeStamp));
      if(m_OdomLimelight.getDerivationConfidence(m_swerve,currentPose,currentTimeStamp)){
        m_swerve.addVisionMeasurement(currentPose,Utils.getCurrentTimeSeconds());
      }
    }*/
  }

  public void AutonMode(){
  }

  public void TeleopMode(){
    homeRobot();
  }

  public void TeleopPeriodic(){
    refreshSmartDashboard();
    GetPIDValues();

    /*if(m_OdomLimelight.isAnyTargetAvailable()){
      m_OdomLimelight.SetRobotOrientation(m_swerve.getState().Pose.getRotation().getDegrees(),0);
  
      Pose2d currentPose=m_OdomLimelight.GetPoseViaMegatag2();
      double currentTimeStamp=Utils.getCurrentTimeSeconds();
      m_OdomLimelight.TrimPoseArray(3);

      //System.out.println(m_OdomLimelight.getDerivationConfidence(m_swerve,currentPose,currentTimeStamp));
      //if(m_OdomLimelight.getDerivationConfidence(m_swerve,currentPose,currentTimeStamp)){
        m_swerve.addVisionMeasurement(currentPose,currentTimeStamp);
      //}
    } */
  }

  public void AllPeriodic(){
    m_Field2d.setRobotPose(m_swerve.getPose2d());
    SmartDashboard.putNumber("Match Time", DriverStation.getMatchTime()); //elastic
    SmartDashboard.putNumber("Voltage",RobotController.getBatteryVoltage()); //elastic
    m_JacksonsCoolPanel.isAllCanAvailable(checkCan());
    setScoreLevelPOV(()->m_xBoxOperator.getPOV());
    setLineupSide(()->m_xBoxOperator.getXButtonPressed(),()->m_xBoxOperator.getBButtonPressed());
    //setDoesDoAlgae(()->m_xBoxOperator.getLeftBumperButtonPressed());
  }

  public void resetGyroPoseDisabled(){
    /*if(m_OdomLimelight.isAnyTargetAvailable()){
      Pose2d currentPose=m_OdomLimelight.GetPoseViaMegatag2();
      Rotation2d newRot=(DriverStation.getAlliance().get()==Alliance.Blue)?
                        new Rotation2d().fromDegrees(0):
                        new Rotation2d().fromDegrees(180);
      m_swerve.resetPose(new Pose2d(currentPose.getX(),currentPose.getY(),newRot));
      m_OdomLimelight.SetRobotOrientation(m_swerve.getState().Pose.getRotation().getDegrees(),0);
    }*/
  }

  public void homeRobot(){
    if(!hasHomed){
      hasHomed = true;
      CommandScheduler.getInstance().schedule(new ElevatorHomeCommand(m_elevator));
      //CommandScheduler.getInstance().schedule(new DunkinDonutHomeCommand(m_dunkinDonut));
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


  
  