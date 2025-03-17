package frc.robot;

import com.ctre.phoenix6.Utils;
import com.fasterxml.jackson.databind.ser.std.StdKeySerializers.Default;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.*;
import frc.robot.utilities.JacksonsCoolPanel;
import frc.robot.utilities.Leds;
import frc.robot.utilities.Limelight;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;
import frc.robot.commands.*;
import frc.robot.Constants.AnimationStates;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.ButtonBindings;
import frc.robot.Constants.ClimbConstants;
import frc.robot.Constants.CoolPanelConstants;
import frc.robot.Constants.DunkinDonutConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.InputControllers;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.LEDStates;
import frc.robot.Constants.LedConstants;
import frc.robot.Constants.LimeLightValues;
import frc.robot.Constants.ElevatorConstants.ElevatorScoreLevel;

public class RobotContainer {
  private final Field2d m_Field2d = new Field2d();
  
  private final int translationAxis = XboxController.Axis.kLeftY.value;
  private final int strafeAxis = XboxController.Axis.kLeftX.value;
  private final int rotationAxis = XboxController.Axis.kRightX.value;
  private final CommandXboxController m_xBoxDriver = new CommandXboxController(InputControllers.kXboxDrive);
  private XboxController m_xBoxOperator = new XboxController(InputControllers.kXboxOperator);

  //DunkinSubsystem
  private DunkinDonutSubsystem m_dunkinDonut = new DunkinDonutSubsystem(DunkinDonutConstants.coralCanID,DunkinDonutConstants.algaeCanID1,DunkinDonutConstants.rotateCanID,
                                                                        DunkinDonutConstants.canCoderID,DunkinDonutConstants.limitSwitchDIO,DunkinDonutConstants.coralIndexerID,
                                                                        DunkinDonutConstants.indexerLimitSwitchDIO);
  
  //ElevatorSubsystem
  private ElevatorSubsystem m_elevator = new ElevatorSubsystem(ElevatorConstants.elevatorCanID,ElevatorConstants.canAndColorID);

  //ClimbSubsystem
  //private ClimbSubsystem m_climb=new ClimbSubsystem(ClimbConstants.climbCanID1,ClimbConstants.climbCanID2,ClimbConstants.encoderDIO,ClimbConstants.ServoPMW);

  public final Leds m_LEDS=new Leds(LedConstants.LedCanID,LedConstants.LedCount);
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

  public boolean hasHomed=false;
  public boolean m_isRightLineup=false;
  public boolean m_doAlgae=false;
  public boolean m_hasResetGyro=false;

  public RobotContainer(){
  SmartDashboard.putData("field", m_Field2d);
    
  configureNamedCommands();
  m_autoChooser=AutoBuilder.buildAutoChooser();
  createAutonomousCommandList(); 
  configureButtonBindings();
  resetDefaultCommand();
  }

  public void configureNamedCommands() {
    NamedCommands.registerCommand("AutoScoreLeftL2",new AutoScoreCommand(m_swerve,m_elevator,m_dunkinDonut,()->ElevatorScoreLevel.L2,m_leftLimelight,m_rightLimelight,()->false,()->false));
    NamedCommands.registerCommand("AutoScoreRightL2",new AutoScoreCommand(m_swerve,m_elevator,m_dunkinDonut,()->ElevatorScoreLevel.L2,m_leftLimelight,m_rightLimelight,()->true,()->false));
    NamedCommands.registerCommand("AutoScoreLeftL3",new AutoScoreCommand(m_swerve,m_elevator,m_dunkinDonut,()->ElevatorScoreLevel.L3,m_leftLimelight,m_rightLimelight,()->false,()->false));
    NamedCommands.registerCommand("AutoScoreLeftL3Algae",new AutoScoreCommand(m_swerve,m_elevator,m_dunkinDonut,()->ElevatorScoreLevel.L3,m_leftLimelight,m_rightLimelight,()->false,()->true));
    NamedCommands.registerCommand("AutoScoreRightL3",new AutoScoreCommand(m_swerve,m_elevator,m_dunkinDonut,()->ElevatorScoreLevel.L3,m_leftLimelight,m_rightLimelight,()->true,()->false));
    NamedCommands.registerCommand("AutoScoreLeftL4",new AutoScoreCommand(m_swerve,m_elevator,m_dunkinDonut,()->ElevatorScoreLevel.L4,m_leftLimelight,m_rightLimelight,()->false,()->false));
    NamedCommands.registerCommand("AutoScoreLeftL4Algae",new AutoScoreCommand(m_swerve,m_elevator,m_dunkinDonut,()->ElevatorScoreLevel.L4,m_leftLimelight,m_rightLimelight,()->false,()->true));
    NamedCommands.registerCommand("AutoScoreRightL4",new AutoScoreCommand(m_swerve,m_elevator,m_dunkinDonut,()->ElevatorScoreLevel.L4,m_leftLimelight,m_rightLimelight,()->true,()->false));
    NamedCommands.registerCommand("ToggleIntake", new DunkinDonutCoralCommand(m_dunkinDonut,m_LEDS,m_elevator,IntakeConstants.coralIntakeSpeed,true,true,IntakeConstants.indexerIntakeSpeed));
  }

  private void configureButtonBindings(){
    System.out.println("N0");
    //QOL Drive
    /*ResetGyroCommand resetGyroCommand=new ResetGyroCommand(m_swerve);
    m_xBoxDriver.start().onTrue(resetGyroCommand);*/

    GasPedalCommand gasPedalCommand=new GasPedalCommand(m_swerve,()->m_xBoxDriver.getRightTriggerAxis());
    m_xBoxDriver.rightTrigger().whileTrue(gasPedalCommand);

    //Climb
   // ClimbRotateCommand rotateFwdCommand=new ClimbRotateCommand(m_climb,ClimbConstants.climbForwardSpeed); //0.5 was to strong and bent the shaft...so we decided to up the power XD
   // m_xBoxDriver.a().whileTrue(rotateFwdCommand);

   // ClimbRotateCommand rotateBwdCommand=new ClimbRotateCommand(m_climb,ClimbConstants.climbBackSpeed);
   // m_xBoxDriver.b().whileTrue(rotateBwdCommand);
    
    /*ToggleClimbPID testPID1=new ToggleClimbPID(m_climb,ClimbConstants.climbPos1);
    m_xBoxDriver.leftBumper().onTrue(testPID1);

    ToggleClimbPID testPID2=new ToggleClimbPID(m_climb,ClimbConstants.climbPos2);
    m_xBoxDriver.rightBumper().onTrue(testPID2);*/
    System.out.println("N1");
    //DunkinDonutCommands
   DunkinDonutRotateCommand DunkinRotateCommand=new DunkinDonutRotateCommand(m_dunkinDonut,()->m_xBoxOperator.getRightX()*0.5);
    Trigger operatorRightXJoystick=new Trigger(()->Math.abs(m_xBoxOperator.getRightX())>Constants.ButtonBindings.joystickDeadband);
    operatorRightXJoystick.whileTrue(DunkinRotateCommand);
    System.out.println("N2");
    ClawPIDScoreIntake clawProcessorScore=new ClawPIDScoreIntake(m_dunkinDonut,m_elevator,ElevatorConstants.processorScorePos,DunkinDonutConstants.processorScorePos,DunkinDonutConstants.processorScoreSpeed,
                                                                ElevatorConstants.elevatorLowLimit,DunkinDonutConstants.rotateHomePos);
    Trigger processorRightTrigger=new Trigger(()->Math.abs(m_xBoxOperator.getRightTriggerAxis())>ButtonBindings.joystickDeadband);
    processorRightTrigger.whileTrue(clawProcessorScore);

    ClawPIDScoreIntake clawLollipopIntake=new ClawPIDScoreIntake(m_dunkinDonut,m_elevator,ElevatorConstants.lollipopGrabPos,DunkinDonutConstants.lollipopGrabPos,DunkinDonutConstants.lollipopGrabSpeed,
                                                              ElevatorConstants.elevatorLowLimit,DunkinDonutConstants.algaeStowPos);
    Trigger lollipopIntakeLeftTrigger=new Trigger(()->Math.abs(m_xBoxOperator.getLeftTriggerAxis())>ButtonBindings.joystickDeadband);
    lollipopIntakeLeftTrigger.whileTrue(clawLollipopIntake);
    /*ClawPIDScoreIntake clawGroundIntake=new ClawPIDScoreIntake(m_dunkinDonut,m_elevator,ElevatorConstants.groundIntakePos,DunkinDonutConstants.groundIntakePos,DunkinDonutConstants.groundIntakeSpeed,
                                                              ElevatorConstants.elevatorLowLimit,DunkinDonutConstants.algaeStowPos);
    Trigger clawGroundIntakeLeftTrigger=new Trigger(()->Math.abs(m_xBoxOperator.getLeftTriggerAxis())>ButtonBindings.joystickDeadband);
    clawGroundIntakeLeftTrigger.whileTrue(clawGroundIntake);*/
    DunkinDonutCoralCommand DunkinCoralCommandIntake = new DunkinDonutCoralCommand(m_dunkinDonut,m_LEDS, m_elevator,IntakeConstants.coralIntakeSpeed, true, true,IntakeConstants.indexerIntakeSpeed);
    JoystickButton operatorDunkinCoralButtonIntake = new JoystickButton(m_xBoxOperator,Constants.ButtonBindings.dunkinCoralIntakeButton);
    operatorDunkinCoralButtonIntake.onTrue(DunkinCoralCommandIntake);
    DunkinDonutCoralCommand DunkinCoralCommand = new DunkinDonutCoralCommand(m_dunkinDonut,m_LEDS,IntakeConstants.coralOutakeSpeed,false, true,IntakeConstants.indexerOuttakeSpeed);
    JoystickButton operatorDunkinCoralButton = new JoystickButton(m_xBoxOperator,Constants.ButtonBindings.dunkinCoralOutakeButton);
    operatorDunkinCoralButton.whileTrue(DunkinCoralCommand);
    
    Command stowCommand = new DunkinDonutRotatePIDCommand(m_dunkinDonut,DunkinDonutConstants.rotateHomePos);
    JoystickButton operatorStowCommand = new JoystickButton(m_xBoxOperator, Constants.ButtonBindings.operatorStow);
    operatorStowCommand.onTrue(stowCommand);

    //ElevatorCommands
    ElevatorCommand ElevatorCommand = new ElevatorCommand(m_elevator, () -> m_xBoxOperator.getLeftY()*0.5);
    Trigger operatorLeftYJoystick = new Trigger(()->Math.abs(m_xBoxOperator.getLeftY())>Constants.ButtonBindings.joystickDeadband);
    operatorLeftYJoystick.whileTrue(ElevatorCommand);
    
    AutoScoreCommand seqScoreCommandManual=new AutoScoreCommand(m_swerve,m_elevator,m_dunkinDonut,()->m_currentScoreLevel,()->m_doAlgae,true);
    JoystickButton operatorSeqScoreManualButton=new JoystickButton(m_xBoxOperator,Constants.ButtonBindings.elevatorScoreManualButton);
    operatorSeqScoreManualButton.onTrue(seqScoreCommandManual);
    AutoScoreCommand seqScoreCommandAuto = new AutoScoreCommand(m_swerve,m_elevator,m_dunkinDonut,()->m_currentScoreLevel,m_leftLimelight,m_rightLimelight,()->m_isRightLineup,()->m_doAlgae);
    JoystickButton operatorSeqScoreAuto = new JoystickButton(m_xBoxOperator,Constants.ButtonBindings.elevatorScoreAutoButton);
    operatorSeqScoreAuto.onTrue(seqScoreCommandAuto);
  }

  public void setScoreLevelPOV(Supplier<Integer> pov){
    switch(pov.get()){
      case 90:
      m_currentScoreLevel=ElevatorScoreLevel.L1;
      break;
      case 180:
      m_currentScoreLevel=ElevatorScoreLevel.L2;
      break;
      case 270:
      m_currentScoreLevel=ElevatorScoreLevel.L3;
      break;
      case 0:
      m_currentScoreLevel=ElevatorScoreLevel.L4;
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

  public void setDoesDoAlgae(Supplier<Boolean> buttonPress){
    if(buttonPress.get()){
      m_doAlgae=m_doAlgae?false:true;
    }
  } 
      
  private void refreshSmartDashboard(){  
    SmartDashboard.putNumber("Elevator Relative",m_elevator.getRelativePos());
    SmartDashboard.putBoolean("m_ElevatorLevel1", m_currentScoreLevel==ElevatorScoreLevel.L1);
    SmartDashboard.putBoolean("m_ElevatorLevel2", m_currentScoreLevel==ElevatorScoreLevel.L2);
    SmartDashboard.putBoolean("m_ElevatorLevel3", m_currentScoreLevel==ElevatorScoreLevel.L3);
    SmartDashboard.putBoolean("m_ElevatorLevel4", m_currentScoreLevel==ElevatorScoreLevel.L4);
    SmartDashboard.putBoolean("LEFT Lineup",m_isRightLineup==false);
    SmartDashboard.putBoolean("RIGHT Lineup",m_isRightLineup==true);
    SmartDashboard.putBoolean("Do Algea", m_doAlgae);
    SmartDashboard.putNumber("Dunkin Rotate ABS",m_dunkinDonut.getAbsoluteEncoderPos());
    SmartDashboard.putString("Current Score Level",m_currentScoreLevel.toString());
    //SmartDashboard.putNumber("Climb ABS Pos",m_climb.getAbsoluteEncoderPos());
    SmartDashboard.putNumber("LEFT LIMELIGHT DISTANCE",m_leftLimelight.getDistance());
    SmartDashboard.putNumber("RIGHT LIMELIGHT DISTANCE",m_rightLimelight.getDistance());
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
    return (()->(m_swerve.checkCANConnections()&&m_dunkinDonut.checkCANConnections()&&m_elevator.checkCANConnections()));//m_climb.checkCANConnections()));
  }

  public void DisabledInit(){
    m_swerve.setVisionMeasurementStdDevs(VecBuilder.fill(5,5,9999999));
    if(!m_hasResetGyro){
      m_hasResetGyro=true;
      m_swerve.getPigeon2().reset();
    }
  }
   
  public void DisabledPeriodic(){
    if(m_rightLimelight.isAnyTargetAvailable()){
      m_LEDS.setLedState(LEDStates.DISABLEDSEETARGET,false);
    }else{
      m_LEDS.setLedState(LEDStates.DISABLEDERROR,false);
    }

    if(m_rightLimelight.isAnyTargetAvailable()){
      m_rightLimelight.SetRobotOrientation(m_swerve.getPigeon2().getRotation2d().getDegrees(),0);
  
      Pose2d currentPose=m_rightLimelight.GetPoseViaMegatag2();
      double currentTimeStamp=Utils.getCurrentTimeSeconds();


      m_swerve.addVisionMeasurement(currentPose,currentTimeStamp);
    } 
    /* m_LEDS.setLedState(LEDStates.DISABLED);
    m_LEDS.activateLEDS();*/
  }
  
  public void AutoPeriodic(){
    if(m_rightLimelight.isAnyTargetAvailable()){
      m_rightLimelight.SetRobotOrientation(m_swerve.getPigeon2().getRotation2d().getDegrees(),0);
  
      Pose2d currentPose=m_rightLimelight.GetPoseViaMegatag2();
      double currentTimeStamp=Utils.getCurrentTimeSeconds();


      m_swerve.addVisionMeasurement(currentPose,currentTimeStamp);
    } 
  }

  public void AutonMode(){
    homeRobot();
  }

  public void TeleopMode(){
    homeRobot();
    m_LEDS.setLedState(LEDStates.TELEOP,false);
    
  }
  
  public void updateLEDs(){
    boolean m_timerStarted = false;
    double m_startTime = 0;
    double m_runTime = 3;
    double x = 0;
    
    try {

      //overwrite has has coral state using module level algea
      if(m_LEDS.getLedState()==LEDStates.HASCORAL && m_doAlgae){
        m_LEDS.setLedState(LEDStates.HASCORALANDDOALGEA,false);
      }

      if(m_isRightLineup){
      if(m_leftLimelight.isAnyTargetAvailable()){
        x=Math.abs(m_leftLimelight.getX());
        if(x<LimeLightValues.leftLineupXDeadband){
          m_LEDS.setLedState(LEDStates.LINEDUP,false);
        }
      }
    }else{
    }if(m_rightLimelight.isAnyTargetAvailable()){
        x=Math.abs(m_rightLimelight.getX());
        if(x<LimeLightValues.rightLineupXDeadband){
          m_LEDS.setLedState(LEDStates.LINEDUP,false);
        }
      }

    switch(m_LEDS.getLedState()){
      case ENABLED:
        //m_LEDS.activateLEDS();
      break;
      case GOLEFT:
      if(!m_timerStarted){
        m_timerStarted = true;
        m_startTime = Timer.getFPGATimestamp();
      }
      if (m_startTime + m_runTime < Timer.getTimestamp()){
        //m_LEDS.activateLEDS();
        
      }else{
          m_LEDS.setLedState(LEDStates.ENABLED,false);
      }
      break;
      case GORIGHT:
      if(!m_timerStarted){
        m_timerStarted = true;
        m_startTime = Timer.getFPGATimestamp();
      }
      if (m_startTime + m_runTime < Timer.getTimestamp()){
        //m_LEDS.activateLEDS();
      }else{
          m_LEDS.setLedState(LEDStates.ENABLED,false);
      }
      break;
      case AUTOSCORING:
      if(!m_timerStarted){
        m_timerStarted = true;
        m_startTime = Timer.getFPGATimestamp();
      }
      if (m_startTime + m_runTime < Timer.getTimestamp()){
       // m_LEDS.activateLEDS();
      }else{
          m_LEDS.setLedState(LEDStates.ENABLED,false);
      }
      break;
      case HASCORAL:
      if(!m_timerStarted){
        m_timerStarted = true;
        m_startTime = Timer.getFPGATimestamp();
      }
      if (m_startTime + m_runTime < Timer.getTimestamp()){
      //  m_LEDS.activateLEDS();
      }else{
      //    m_LEDS.setLedState(LEDStates.ENABLED);

      }
      break;
<<<<<<< HEAD
           case LINEDUP:
=======
      case HASCORALANDDOALGEA:
      if(!m_timerStarted){
        m_timerStarted = true;
        m_startTime = Timer.getFPGATimestamp();
      }
      if (m_startTime + m_runTime < Timer.getTimestamp()){
     //   m_LEDS.activateLEDS();
      }else{
          m_LEDS.setLedState(LEDStates.ENABLED,false);
      }
      break;
      case LINEDUP:
>>>>>>> ba44c1fb142d76003584f7004717d4a04aedda0d
      if(!m_timerStarted){
        m_timerStarted = true;
        m_startTime = Timer.getFPGATimestamp();
      }
      if (m_startTime + m_runTime < Timer.getTimestamp()){
       // m_LEDS.activateLEDS();
      }else{
          m_LEDS.setLedState(LEDStates.ENABLED,false);
      }
      break;
      case INTAKING:
      if(!m_timerStarted){
        m_timerStarted = true;
        m_startTime = Timer.getFPGATimestamp();
      }
      if (m_startTime + m_runTime < Timer.getTimestamp()){
       // m_LEDS.activateLEDS();
      }else{
          m_LEDS.setLedState(LEDStates.ENABLED,false);
      }
      break;
    }

  }
  catch (Exception e){
    System.out.println("LED Update Error");
    }
  }
  public void TeleopPeriodic(){
    refreshSmartDashboard();
    updateLEDs();
   
    if(m_rightLimelight.isAnyTargetAvailable()){
      m_rightLimelight.SetRobotOrientation(m_swerve.getPigeon2().getRotation2d().getDegrees(),0);
  
      Pose2d currentPose=m_rightLimelight.GetPoseViaMegatag2();
      double currentTimeStamp=Utils.getCurrentTimeSeconds();


      m_swerve.addVisionMeasurement(currentPose,currentTimeStamp);
    } 
  }

  public void AllPeriodic(){
    m_Field2d.setRobotPose(m_swerve.getPose2d());
    SmartDashboard.putNumber("Match Time", DriverStation.getMatchTime()); //elastic
    SmartDashboard.putNumber("Voltage",RobotController.getBatteryVoltage()); //elastic
    m_JacksonsCoolPanel.isAllCanAvailable(checkCan());
    setScoreLevelPOV(()->m_xBoxOperator.getPOV());
    setLineupSide(()->m_xBoxOperator.getXButtonPressed(),()->m_xBoxOperator.getBButtonPressed());
    setDoesDoAlgae(()->m_xBoxOperator.getStartButtonPressed());
  }

  public void homeRobot(){
    if(!hasHomed){
      hasHomed = true;
      CommandScheduler.getInstance().schedule(new ElevatorHomeCommand(m_elevator));
      CommandScheduler.getInstance().schedule(new DunkinDonutHomeCommand(m_dunkinDonut));
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


  
  