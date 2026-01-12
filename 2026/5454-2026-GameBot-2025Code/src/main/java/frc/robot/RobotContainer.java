package frc.robot;
import com.ctre.phoenix6.Utils;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
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
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.ButtonBindings;
import frc.robot.Constants.CoolPanelConstants;
import frc.robot.Constants.DunkinDonutConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.GroundIntakeConstants;
import frc.robot.Constants.InputControllers;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.LEDStates;
import frc.robot.Constants.LedConstants;
import frc.robot.Constants.LimeLightValues;
import frc.robot.Constants.ElevatorConstants.ElevatorScoreLevel;
import edu.wpi.first.wpilibj2.command.InstantCommand;
public class RobotContainer {
  private final Field2d m_Field2d = new Field2d();
  
  private final int translationAxis = XboxController.Axis.kLeftY.value;
  private final int strafeAxis = XboxController.Axis.kLeftX.value;
  private final int rotationAxis = XboxController.Axis.kRightX.value;

  private final CommandXboxController m_xBoxDriver = new CommandXboxController(InputControllers.kXboxDrive);
  private XboxController m_xBoxOperator = new XboxController(InputControllers.kXboxOperator);
  public final Leds m_LEDS=new Leds(LedConstants.LedCanID,LedConstants.LedCount);
  public final CommandSwerveDrivetrain m_swerve = TunerConstants.createDrivetrain();
  public final Limelight m_leftLimelight=new Limelight(Constants.LimeLightValues.leftLimelightHeight,Constants.LimeLightValues.leftLimelightAngle,
                                                0,Constants.LimeLightValues.leftLimelightName);
  public final Limelight m_rightLimelight=new Limelight(Constants.LimeLightValues.rightLimelightHeight,Constants.LimeLightValues.rightLimelightAngle,
                                                0,Constants.LimeLightValues.rightLimelightName);

 
  private final SendableChooser<Command> m_autoChooser;
 
  public boolean hasHomed=false;
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

  }

  private void configureButtonBindings(){
    //QOL Drive
    /*ResetGyroCommand resetGyroCommand=new ResetGyroCommand(m_swerve);
    m_xBoxDriver.start().onTrue(resetGyroCommand);*/

    GasPedalCommand gasPedalCommand=new GasPedalCommand(m_swerve,()->m_xBoxDriver.getRightTriggerAxis());
    m_xBoxDriver.rightTrigger().whileTrue(gasPedalCommand);

 
  }
     
  private void refreshSmartDashboard(){  
    try{
      SmartDashboard.putNumber("LEFT LIMELIGHT DISTANCE",m_leftLimelight.getDistance());
      SmartDashboard.putNumber("RIGHT LIMELIGHT DISTANCE",m_rightLimelight.getDistance());
    }catch(Exception e){}

  }
  
  private void createAutonomousCommandList(){
    try{
      SmartDashboard.putData("Auto Chooser",m_autoChooser);

    }catch(Exception e){
      System.out.println("Create Autos Failed, Exception: " + e.getMessage());
    }
  }


  public BooleanSupplier checkCan(){
    return (()->true);
    //return (()->(m_swerve.checkCANConnections()&&m_dunkinDonut.checkCANConnections()&&m_elevator.checkCANConnections()));//m_climb.checkCANConnections()));
  }

  public void DisabledInit(){
    m_swerve.setVisionMeasurementStdDevs(VecBuilder.fill(1,1,99999999));
    if(!m_hasResetGyro){
      m_hasResetGyro=true;
      m_swerve.getPigeon2().reset();
    }
  }
   
  public void DisabledPeriodic(){
    /*if(m_rightLimelight.isAnyTargetAvailable()||m_leftLimelight.isAnyTargetAvailable()){
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

    if(m_leftLimelight.isAnyTargetAvailable()){
      m_leftLimelight.SetRobotOrientation(m_swerve.getPigeon2().getRotation2d().getDegrees(),0);
  
      Pose2d currentPose=m_leftLimelight.GetPoseViaMegatag2();
      double currentTimeStamp=Utils.getCurrentTimeSeconds();


      m_swerve.addVisionMeasurement(currentPose,currentTimeStamp);
    } */
    m_LEDS.setLedState(LEDStates.DISABLED,false);
    //m_LEDS.activateLEDS();
  }
  
  public void AutoPeriodic(){
    /*if(m_rightLimelight.isAnyTargetAvailable()){
      m_rightLimelight.SetRobotOrientation(m_swerve.getPigeon2().getRotation2d().getDegrees(),0);
  
      Pose2d currentPose=m_rightLimelight.GetPoseViaMegatag2();
      double currentTimeStamp=Utils.getCurrentTimeSeconds();


      m_swerve.addVisionMeasurement(currentPose,currentTimeStamp);
    } 

    if(m_leftLimelight.isAnyTargetAvailable()){
      m_leftLimelight.SetRobotOrientation(m_swerve.getPigeon2().getRotation2d().getDegrees(),0);
  
      Pose2d currentPose=m_leftLimelight.GetPoseViaMegatag2();
      double currentTimeStamp=Utils.getCurrentTimeSeconds();


      m_swerve.addVisionMeasurement(currentPose,currentTimeStamp);
    } */
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
    double rawX = 0;
    
    try {

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

    if(m_leftLimelight.isAnyTargetAvailable()){
      m_leftLimelight.SetRobotOrientation(m_swerve.getPigeon2().getRotation2d().getDegrees(),0);
  
      Pose2d currentPose=m_leftLimelight.GetPoseViaMegatag2();
      double currentTimeStamp=Utils.getCurrentTimeSeconds();


      m_swerve.addVisionMeasurement(currentPose,currentTimeStamp);
    } 
  }

  public void AllPeriodic(){
    m_Field2d.setRobotPose(m_swerve.getPose2d());
    SmartDashboard.putNumber("Match Time", DriverStation.getMatchTime()); //elastic
    SmartDashboard.putNumber("Voltage",RobotController.getBatteryVoltage()); //elastic
    refreshSmartDashboard();
  }

  public void homeRobot(){
    if(!hasHomed){
      hasHomed = true;
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


  
  