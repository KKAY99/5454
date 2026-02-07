package frc.robot;
import com.ctre.phoenix6.Utils;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.*;
import frc.robot.subsystems.shooter.NewShooterSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.utilities.JacksonsCoolPanel;
import frc.robot.utilities.Leds;
import frc.robot.utilities.Limelight;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import org.ejml.dense.row.mult.MatrixMatrixMult_MT_ZDRM;

import frc.robot.commands.*;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.ButtonBindings;
import frc.robot.Constants.CoolPanelConstants;
import frc.robot.Constants.HoodConstants;
import frc.robot.Constants.InputControllers;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.LEDStates;
import frc.robot.Constants.LedConstants;
import frc.robot.Constants.LimeLightValues;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.TurretConstants;
import frc.robot.subsystems.TurretSubsystem;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.shooter.ShotCalculator;
import frc.robot.subsystems.shooter.PassCalculator.ShootingParameters;
public class RobotContainer {
  private final Field2d m_Field2d = new Field2d();
  
  private final int translationAxis = XboxController.Axis.kLeftY.value;
  private final int strafeAxis = XboxController.Axis.kLeftX.value;
  private final int rotationAxis = XboxController.Axis.kRightX.value;

  private final CommandXboxController m_xBoxDriver = new CommandXboxController(InputControllers.kXboxDrive);
  private CommandXboxController m_xBoxOperator = new CommandXboxController(InputControllers.kXboxOperator);
  private CommandXboxController m_CustomController = new CommandXboxController(InputControllers.kCustomController);
  private CommandXboxController m_FunnyController = new CommandXboxController(InputControllers.kFunnyController);
  //public final Leds m_LEDS=new Leds(LedConstants.LedCanID,LedConstants.LedCount);
  public final CommandSwerveDrivetrain m_swerve = TunerConstants.createDrivetrain();
  public final IntakeSubsystem m_intake = new IntakeSubsystem(Constants.IntakeConstants.IntakeMotorCanID,Constants.IntakeConstants.LowMotorCanID);
  public final NewShooterSubsystem m_newShooter = new NewShooterSubsystem(Constants.NewShooterConstants.shooter1CANID,
                                                        Constants.NewShooterConstants.shooter2CANID,
                                                         Constants.NewShooterConstants.kickerCANID,
                                                         Constants.NewShooterConstants.hoodCANID);

  public final ShooterSubsystem m_shooter = new ShooterSubsystem(Constants.ShooterConstants.ShooterCanID,
                                                                 Constants.ShooterConstants.KickerCanID,
                                                                 Constants.ShooterConstants.IdleSpeed);
  public final HopperSubsystem m_hopper = new HopperSubsystem(Constants.HopperConstants.HopperMotorCanID);
  public final TurretSubsystem m_TurretSubsystem = new TurretSubsystem(Constants.TurretConstants.turretCanID);
  public final Limelight m_leftLimelight=new Limelight(Constants.LimeLightValues.leftLimelightHeight,Constants.LimeLightValues.leftLimelightAngle,
                                                0,Constants.LimeLightValues.leftLimelightName);
  public final Limelight m_rightLimelight=new Limelight(Constants.LimeLightValues.rightLimelightHeight,Constants.LimeLightValues.rightLimelightAngle,
                                                0,Constants.LimeLightValues.rightLimelightName);

 
  private final SendableChooser<Command> m_autoChooser;


  public Command Left2Neutral() {
    // This method loads the auto when it is called, however, it is recommended
    // to first load your paths/autos when code starts, then return the
    // pre-loaded auto/path
    return new PathPlannerAuto("Left2Neutral");
  }
  public Command Right2Left() {
    // This method loads the auto when it is called, however, it is recommended
    // to first load your paths/autos when code starts, then return the
    // pre-loaded auto/path
    return new PathPlannerAuto("Right2Left");
  }
 
  public boolean hasHomed=false;
  public boolean m_hasResetGyro=false;
  private String m_activeHub="Undefined";
  private String m_startHub="";
  private double m_activeHubTime=99999;
  private String m_hubMatch="Undefined";
  private String m_activeHubPhase="Undefined";
  private ShotCalculator m_ShotCalculator = new ShotCalculator();
  
  public RobotContainer(){
    SmartDashboard.putData("field", m_Field2d);
    configureNamedCommands();
    m_autoChooser=AutoBuilder.buildAutoChooser();
    createAutonomousCommandList(); 
    configureButtonBindings();
    resetDefaultCommand();
    //m_swerve.playMusic("Indiana.chrp");
  }


    //Named Commands
  public void configureNamedCommands() {
    NamedCommands.registerCommand("agitateon", m_hopper.agitateonCommand());
    NamedCommands.registerCommand("agitateoff", m_hopper.agitateoffCommand());
    NamedCommands.registerCommand("intakeon", m_intake.intakeonCommand());
    NamedCommands.registerCommand("intakeoff", m_intake.intakeoffCommand());
    NamedCommands.registerCommand("OLDshootoff", m_shooter.OldShootonCommand());
    NamedCommands.registerCommand("OLDshooton", m_shooter.OldShootoffCommand());
    NamedCommands.registerCommand("NEWshooton ", m_newShooter.shootonCommand());
    NamedCommands.registerCommand("NEWshootoff", m_newShooter.shootoffCommand());
  }

  private void configureButtonBindings(){
    //QOL Drive
    /*ResetGyroCommand resetGyroCommand=new ResetGyroCommand(m_swerve);
    m_xBoxDriver.start().onTrue(resetGyroCommand);*/
    //using CommandXBox for clarity 
    GasPedalCommand gasPedalCommand=new GasPedalCommand(m_swerve,()->m_xBoxDriver.getRightTriggerAxis());
    m_xBoxDriver.rightTrigger().whileTrue(gasPedalCommand);

    Command agitate = m_hopper.agitateCommand();
    m_CustomController.a().whileTrue(agitate);

    Command intake = m_intake.intakeCommand();
    m_CustomController.y().whileTrue(intake);
    m_CustomController.x().toggleOnTrue(intake);
    Command shoot = m_shooter.OldShootCommand();
    m_CustomController.b().whileTrue(shoot);
    //Command shoot2 = Commands.startEnd(     ()->m_shooter.runShooter(.75,-1),
    //                                       ()->m_shooter.stopShooter(),
    //                                       m_shooter);
   // m_CustomController.x().whileTrue(shoot2);
  //  Command shoot3 = Commands.startEnd(     ()->m_shooter.runShooter(0.5,-1),
  //                                         ()->m_shooter.stopShooter(),
  //                                         m_shooter);
//    m_CustomController.y().whileTrue(shoot3);              

    m_CustomController.rightBumper().onTrue(Left2Neutral());
    Command resetPose = Commands.run(()->makefalsestartPose(),m_swerve);
    m_CustomController.leftBumper().onTrue(Right2Left());
  
    Command doNothing = Commands.none();
   
    m_xBoxDriver.a().whileTrue(doNothing);
    m_xBoxDriver.b().whileTrue(doNothing);
    m_xBoxDriver.x().whileTrue(doNothing);
    m_xBoxDriver.y().whileTrue(doNothing);
    m_xBoxDriver.rightBumper().whileTrue(doNothing);
    m_xBoxDriver.leftBumper().whileTrue(doNothing);
    
    m_xBoxOperator.a().whileTrue(doNothing);
    m_xBoxOperator.b().whileTrue(doNothing);
    m_xBoxOperator.x().whileTrue(doNothing);
    m_xBoxOperator.y().whileTrue(doNothing);
    m_xBoxOperator.rightBumper().whileTrue(doNothing);
    m_xBoxOperator.leftBumper().whileTrue(doNothing);

    m_FunnyController.rightBumper().whileTrue(agitate);
    Command newShoot = m_newShooter.shootCommand();
    m_FunnyController.a().whileTrue(newShoot);
    Command newHoodUp = m_newShooter.HoodUp();
    m_FunnyController.b().whileTrue(newHoodUp);
    Command newHoodDown = m_newShooter.HoodDown();
    m_FunnyController.x().whileTrue(newHoodDown);
    Command newVelocityShot = Commands.startEnd(  ()->m_newShooter.runShooterVelocity(ShooterConstants.shooterRPM),
                                           ()->m_newShooter.stopNewShooter(),
                                           m_newShooter);
    m_FunnyController.leftBumper().whileTrue(newVelocityShot);
    m_FunnyController.povRight().whileTrue(Commands.startEnd( ()->m_TurretSubsystem.moveTurret(TurretConstants.turretSpeed),
                                                              ()->m_TurretSubsystem.stopTurret(),
                                                              m_TurretSubsystem));
    m_FunnyController.povLeft().whileTrue(Commands.startEnd( ()->m_TurretSubsystem.moveTurret(-TurretConstants.turretSpeed),
                                                              ()->m_TurretSubsystem.stopTurret(),
                                                              m_TurretSubsystem));
  
  }

  private void updateisHubMatched(int Shift){
   Optional<Alliance> ally = DriverStation.getAlliance();
   String currentAlliance="";
   if (ally.isPresent()) {
        if (ally.get() == Alliance.Red) {
            currentAlliance="Red";
        }
        if (ally.get() == Alliance.Blue) {
            currentAlliance="Blue";
        }
  }

  if((m_startHub=="B") && (Shift==2 || Shift==4)){
    //Blue Hub is Active
    m_activeHub="Blue";
    if(currentAlliance=="Blue"){
        m_hubMatch="True";
    }else {
        m_hubMatch="False";
    }
  } else {
    //RED Hub is Active
    m_activeHub="Red";
    if(currentAlliance=="Red"){
        m_hubMatch="True";
    }else {
        m_hubMatch="False";
    }
  }
  
  }
  private void updateHubTime(){
  double secondsRemaining= DriverStation.getMatchTime();
  final int kTransitionEnd=130;
  final int kShift1End=105;
  final int kShift2End=80;
  final int kShift3End=55;
  final int kShift4End=30;
  final int kEndGame=0;
  if(DriverStation.isTeleop())
  { if(secondsRemaining<=0){ // not in FMS or Match Mode
      m_activeHubPhase="Not in Match";
      m_activeHubTime=99999;
      m_activeHub="Both";
      m_hubMatch="True";
    }else if(secondsRemaining>kTransitionEnd){
      m_activeHubPhase="Transition Shift";
      m_activeHubTime=secondsRemaining-kTransitionEnd;
      m_activeHub="Both";
      m_hubMatch="True";
    }else if(secondsRemaining>kShift1End) {
      m_activeHubPhase="Shift 1";
      m_activeHubTime=secondsRemaining-kShift1End;
      updateisHubMatched(1);
    }else if(secondsRemaining>kShift2End){
      m_activeHubPhase="Shift 2";
      m_activeHubTime=secondsRemaining-kShift2End;
      updateisHubMatched(2);
    }else if(secondsRemaining>kShift3End){
      m_activeHubPhase="Shift 3";
      m_activeHubTime=secondsRemaining-kShift3End;
      updateisHubMatched(3);
    }else if(secondsRemaining>kShift4End){
      m_activeHubPhase="Shift 4";
      m_activeHubTime=secondsRemaining-kShift4End;
      updateisHubMatched(4);
    }else { //default to end game if in endgame
      m_activeHubPhase="End Game";
      m_activeHubTime=secondsRemaining;
      m_activeHub="Both";
      m_hubMatch="True";
    }
    
  }else {
    updateUndefinedHub();
  }
  

  }
  private void updateUndefinedHub(){
    m_activeHub="Undefined";
    m_hubMatch="Undefined";
    m_activeHubTime=99999;

  }
  private void updateHubStatus(){
    if((m_activeHub=="")|| (m_activeHub=="Unknown")){
          String gameData;
          gameData = DriverStation.getGameSpecificMessage();
          if(gameData.length() > 0) 
          {
            m_startHub=Character.toString(gameData.charAt(0));
            switch (m_startHub)
          {
            case "B" :
              //Blue case code
              m_startHub="B";
              break;
            case "R" :
              //Red case code
              m_startHub="R";
              break;
            default :
              
              updateUndefinedHub();
              break;
          }
            updateHubTime();
        
      } else {
          updateUndefinedHub();
  
     
     }
    }else {
      updateHubTime();;
    }
  }
  private void refreshSmartDashboard(){  
    try{
      updateHubStatus();
      SmartDashboard.putString("Our Hub Active",m_hubMatch);      
      SmartDashboard.putString("ActiveHub",m_activeHub);
      SmartDashboard.putString("Active Phase",m_activeHubPhase);
      SmartDashboard.putNumber("Active Phase Time",m_activeHubTime);
      
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
    //m_LEDS.setLedState(LEDStates.DISABLED,false);
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

  public void makefalsestartPose(){
    
  Pose2d startPose = new Pose2d(6,7, Rotation2d.fromDegrees(0));
  m_swerve.addVisionMeasurement(startPose,Utils.getCurrentTimeSeconds());
  }
  public Command makeAutoCommandPPTest(){

    // Since we are using a holonomic drivetrain, the rotation component of this pose
// represents the goal holonomic rotation
Pose2d targetPose = new Pose2d(12, 1, Rotation2d.fromDegrees(0));

// Create the constraints to use while pathfinding
PathConstraints constraints = new PathConstraints(
        3.0, 4.0,
        Units.degreesToRadians(540), Units.degreesToRadians(720));

// Since AutoBuilder is configured, we can use it to build pathfinding commands
Command pathfindingCommand = AutoBuilder.pathfindToPose(
        targetPose,
        constraints,
        0.0 // Goal end velocity in meters/sec
);
return pathfindingCommand;
}
  public void AutonMode(){
    homeRobot();
  }

  public void TeleopMode(){
    homeRobot();
    
    //m_LEDS.setLedState(LEDStates.TELEOP,false);
    
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
    m_ShotCalculator.clearShootingParameters();
    ShotCalculator.ShootingParameters shootingInfo = m_ShotCalculator.getParameters(m_swerve);
    //System.out.println("Turret Angle: " + shootingInfo.turretAngle());
    //System.out.println("Turret Velocity:" + shootingInfo.turretVelocity());
    
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
    //disabled drive
    m_swerve.setDefaultCommand(m_swerve.applyRequestDrive(m_xBoxDriver,translationAxis,strafeAxis,rotationAxis));
  }
}


  
  