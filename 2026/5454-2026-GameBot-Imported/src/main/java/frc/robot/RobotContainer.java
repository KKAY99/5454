package frc.robot;
import com.ctre.phoenix6.Utils;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.commands.PathPlannerAuto;

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
import frc.robot.subsystems.shooter.FieldConstants;
import frc.robot.subsystems.shooter.NewShooterSubsystem;
// import frc.robot.subsystems.shooter.ShooterSubsystem; // unused
import frc.robot.utilities.Leds;
import frc.robot.utilities.Limelight;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import java.util.List;
import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import org.ejml.dense.row.mult.MatrixMatrixMult_MT_ZDRM;

import frc.robot.commands.*;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.ButtonBindings;
import frc.robot.Constants.ClimbConstants;
import frc.robot.Constants.CoolPanelConstants;
import frc.robot.Constants.HoodConstants;
import frc.robot.Constants.InputControllers;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.LEDStates;
import frc.robot.Constants.LedConstants;
import frc.robot.Constants.LimeLightValues;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.TurretConstants;
import frc.robot.Constants.TurretStates;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.subsystems.shooter.ShotCalculator;
import frc.robot.subsystems.shooter.PassCalculator.ShootingParameters;
public class RobotContainer {
  private final Field2d m_Field2d = new Field2d();
  
  private final int translationAxis = XboxController.Axis.kLeftY.value;
  private final int strafeAxis = XboxController.Axis.kLeftX.value;
  private final int rotationAxis = XboxController.Axis.kRightX.value;

  private final CommandXboxController m_xBoxDriver = new CommandXboxController(InputControllers.kXboxDrive);
  private CommandXboxController m_xBoxOperator = new CommandXboxController(InputControllers.kXboxOperator);
  //private CommandXboxController m_CustomController = new CommandXboxController(InputControllers.kCustomController);
  //private CommandXboxController m_FunnyController = new CommandXboxController(InputControllers.kFunnyController);
  //public final Leds m_LEDS=new Leds(LedConstants.LedCanID,LedConstants.LedCount);
  public final CommandSwerveDrivetrain m_swerve = TunerConstants.createDrivetrain();
  public final IntakeSubsystem m_intake = new IntakeSubsystem(Constants.IntakeConstants.IntakeMotorCanID, Constants.IntakeConstants.FoldMotorCanID);
  public final NewShooterSubsystem m_newShooter = new NewShooterSubsystem(Constants.ShooterConstants.shooter1CANID,
                                                         Constants.ShooterConstants.shooter2CANID,
                                                         Constants.ShooterConstants.kickerCANID,
                                                         Constants.ShooterConstants.hoodCANID);
  public final HopperSubsystem m_hopper = new HopperSubsystem(Constants.HopperConstants.HopperMotorCanID,
                                                         Constants.ShooterConstants.fuelSensorDIO);
  // use the overlap implementation so CANcoder values are read and pushed
  // to Shuffleboard; the old 'Pots' class only used the potentiometer and
  // ignored the encoders.
  public final TurretSubsystemPots m_TurretSubsystem = new TurretSubsystemPots(Constants.TurretConstants.turretCanID,TurretConstants.TurretPOT);
  public final ClimbSubsystem m_climb = new ClimbSubsystem(ClimbConstants.climbCanID1);
  public final Limelight m_turretLimelight=new Limelight(Constants.LimeLightValues.turretLimelightHeight,
                                            Constants.LimeLightValues.turretLimelightAngle,
                                                0,Constants.LimeLightValues.leftLimelightName);
  public final Limelight m_leftLimelight=new Limelight(Constants.LimeLightValues.leftLimelightHeight,Constants.LimeLightValues.leftLimelightAngle,
                                                0,Constants.LimeLightValues.leftLimelightName);
  public final Limelight m_backLimelight=new Limelight(Constants.LimeLightValues.backLimelightHeight,Constants.LimeLightValues.backLimelightAngle,
                                                0,Constants.LimeLightValues.backLimelightName);

 
  private final SendableChooser<Command> m_autoChooser;
  private final SendableChooser<String> m_pathChooser = new SendableChooser<>();


 // Climb Align
  public void rightClimb() {
    Command extend = m_climb.climbUpCommand();
    Command retract = m_climb.climbDownCommand();
    Command align = new ClimbAutoAlign(true, m_swerve);
    CommandScheduler.getInstance().schedule(Commands.sequence(extend, align, retract));
  }

  public void leftClimb() {
    Command extend = m_climb.climbUpCommand();
    Command retract = m_climb.climbDownCommand();
    Command align = new ClimbAutoAlign(false, m_swerve);
    CommandScheduler.getInstance().schedule(Commands.sequence(extend, align, retract));
  }

  // Auto Setup
  public Command Left2Neutral() {
    // This method loads the auto when it is called, however, it is recommended
    // to first load your paths/autos when code starts, then return the
    // pre-loaded auto/path
    return new PathPlannerAuto("Left2Neutral");
  }
  public Command Right2Left() {
    return new PathPlannerAuto("Right2Left");
  }
  public Command ShootDepotShootNZ() {
    return new PathPlannerAuto("ShootDepotShootNZ");
  }
  private void InitialAutonPathfind(){
    //Magic Comment
    if (m_autoChooser == null) {
      SmartDashboard.putString("Asher's Cool Message:", "No Auto Selected");
     // Don't Run anything
    }
    else {
      try {
        // Prefer the pre-built Command selected in the auto chooser
        Command selectedAuto = m_autoChooser.getSelected();
        if (selectedAuto != null) {
          SmartDashboard.putString("Asher's Cool Message:", "Returning selected Command from auto chooser");
        //Command autoCommand =  ( new PathPlannerAuto(selectedAuto));
      
        }

        // Load the path group to obtain the initial pose of the first trajectory.
        // Use suitable constraints here (these only affect loading; path-following will be handled by the PathPlannerAuto command).
        PathConstraints goToConstraints = new PathConstraints(3.0, 4.0,
          Units.degreesToRadians(540), Units.degreesToRadians(720));
        List<PathPlannerPath> group = PathPlannerAuto.getPathGroupFromAutoFile(selectedAuto.getName());

        if (group == null || group.isEmpty()) {
          SmartDashboard.putString("Asher's Cool Message:", "It aint running 2");
          //Do Nothing
        }else {
        
          Pose2d startPose = group.get(0).getStartingHolonomicPose().orElse(group.get(0).getStartingDifferentialPose());
          startPose=FieldConstants.flipIfRed(startPose);
          Command goToStart = AutoBuilder.pathfindToPose(
            startPose,
            goToConstraints,
            0.0
          );

          Command followAuto = new PathPlannerAuto("DepotShooting");

          // go to start pos then call auto
          SmartDashboard.putString("Asher's Cool Message:","should be running sequence");
          //add auto to scheduler
          Pose2d currentPose = m_swerve.getPose2d();
          if(currentPose.getX()!=0 | currentPose.getY()!=0) {
            //disable pathing
            //            CommandScheduler.getInstance().schedule(Commands.sequence(goToStart, followAuto));
            CommandScheduler.getInstance().schedule(Commands.sequence(followAuto));
          
          } else { //no starting pose
            CommandScheduler.getInstance().schedule(Commands.sequence(followAuto));
          }
        }
      } catch (Exception e) {
        // if anything goes wrong (it probably will), fall back to whatever the original chooser provides
        SmartDashboard.putString("Asher's Cool Message:",e.getMessage());
      }
    }
  }
  //
  public boolean hasHomed=false;
  public boolean m_hasResetGyro=false;
  private String m_activeHub="Undefined";
  private String m_startHub="";
  private double m_activeHubTime=99999;
  private boolean m_hubMatch=false;
  private String m_activeHubPhase="Undefined";
  private ShotCalculator m_ShotCalculator = new ShotCalculator();
  

  //
  public RobotContainer(){
    //filter turret limelight on center targets
    m_turretLimelight.setLimelightIDFilter(10,25);
    SmartDashboard.putData("field", m_Field2d);
    SmartDashboard.putNumber("Speed Adjuster:", 1);
    configureNamedCommands();
    m_autoChooser=AutoBuilder.buildAutoChooser();
    createAutonomousCommandList(); 
    configureButtonBindings();
    resetDefaultCommand();

    m_pathChooser.setDefaultOption("Left2Neutral", "Left2Neutral");
    m_pathChooser.addOption("Right2Left", "Right2Left");
    m_pathChooser.addOption("ShootDepotShootNZ", "ShootDepotShootNZ");
    SmartDashboard.putData("Path Chooser", m_pathChooser);
   // m_TurretSubsystem.playMusic("IndianaJones.chrp");
  }


    //Named Commands
  public void configureNamedCommands() {
    //NamedCommands.registerCommand("climbAlignR", new ClimbAutoAlign(true, m_swerve));
    //NamedCommands.registerCommand("climbAlignL", new ClimbAutoAlign(false, m_swerve));
    NamedCommands.registerCommand("expandIntake", Commands.sequence(new InstantCommand(m_intake::SetIntakeOutMode), new IntakeFoldCommand(m_intake))); //this should fix the intake not folding during autos
    NamedCommands.registerCommand("shrinkIntake", new IntakeIntakeCommand(m_intake));
    NamedCommands.registerCommand("agitateon", m_hopper.agitateonCommand());
    NamedCommands.registerCommand("agitateoff", m_hopper.agitateoffCommand());
    NamedCommands.registerCommand("intakeon", m_intake.intakeonCommand());
    NamedCommands.registerCommand("intakeoff", m_intake.intakeoffCommand());
    NamedCommands.registerCommand("NEWshooton", m_newShooter.shootonCommand());
    NamedCommands.registerCommand("NEWshootoff", m_newShooter.shootoffCommand());
    NamedCommands.registerCommand("shootManual", new ShootManualCommand(m_newShooter,m_hopper, m_intake,
          m_turretLimelight,Constants.ShooterConstants.kAgitateTimeLimit,false));
    NamedCommands.registerCommand("turretManualMove", new WaitCommand(2) );
    NamedCommands.registerCommand("turretManualStop", new WaitCommand(2));
    NamedCommands.registerCommand("climbUp", m_climb.climbUpCommand());
    NamedCommands.registerCommand("climbDown", m_climb.climbDownCommand());
    NamedCommands.registerCommand("completeIntake", new CompleteIntakeCommand(m_intake, m_hopper));
    NamedCommands.registerCommand("popcorn", new ShootPopcornCommand(m_newShooter, m_hopper, m_intake, m_TurretSubsystem,m_swerve, null));
    NamedCommands.registerCommand("shotLookUp", new ShotLookupCommand(m_newShooter, m_hopper, m_intake, m_TurretSubsystem, m_turretLimelight, Constants.ShooterConstants.kAgitateTimeLimit, true));
  }

  private void configureButtonBindings(){
    //Internal Robot Triggers
    Trigger feulDetector = new Trigger(() -> m_hopper.getNoFuel());

    //QOL Drive
    /*ResetGyroCommand resetGyroCommand=new ResetGyroCommand(m_swerve);
    m_xBoxDriver.button(9).onTrue(resetGyroCommand);*/
    //using CommandXBox for clarity 
    GasPedalCommand gasPedalCommand=new GasPedalCommand(m_swerve,()->m_xBoxDriver.getRightTriggerAxis());
    m_xBoxDriver.rightTrigger().whileTrue(gasPedalCommand);

    Command CompleteIntake = new CompleteIntakeCommand(m_intake,m_hopper);
    m_xBoxDriver.leftBumper().toggleOnTrue(CompleteIntake);
    m_xBoxOperator.a().toggleOnTrue(CompleteIntake);

    Command foldIn = new IntakeIntakeCommand(m_intake);

    Command foldOut = new IntakeFoldCommand(m_intake);
    m_xBoxDriver.y().onTrue(Commands.sequence(new InstantCommand(m_intake::toggleIntakeMode), foldOut));
    
    Command outtake = m_intake.outtakeCommand();
    m_xBoxDriver.rightBumper().whileTrue(outtake);
    m_xBoxOperator.b().whileTrue(outtake);
    
    Command climbUp = m_climb.climbUpCommand();
    m_xBoxDriver.b().onTrue(climbUp);
    //m_xBoxDriver.b().onTrue(Commands.runOnce(()->rightClimb()));

    Command climbDown = m_climb.climbDownCommand();
    m_xBoxDriver.a().onTrue(climbDown);
    //m_xBoxDriver.a().onTrue(Commands.runOnce(()->leftClimb()));

    Command shootMapping = new ShootMappingCommand(m_newShooter,m_hopper,m_intake,
                                m_turretLimelight,Constants.ShooterConstants.kAgitateTimeLimit,true);
    m_xBoxOperator.rightTrigger().whileTrue(shootMapping);

    Command shootManual = new ShotLookupCommand(m_newShooter, m_hopper, m_intake, m_TurretSubsystem,
                                m_turretLimelight, Constants.ShooterConstants.kAgitateTimeLimit, true);
    m_xBoxDriver.start().whileTrue(shootManual);
    m_xBoxOperator.leftTrigger().whileTrue(shootManual);

    Command shootPopcorn = new ShootPopcornCommand(m_newShooter, m_hopper, m_intake, m_TurretSubsystem, m_swerve, null);
    m_xBoxDriver.leftTrigger().whileTrue(shootPopcorn);

    /*Command shootKernelCommand = new ShootKernelCommand(m_newShooter,m_hopper,m_intake,Constants.ShooterConstants.kAgitateTimeLimit,true,m_TurretSubsystem,null);
    m_xBoxDriver.x().whileTrue(shootKernelCommand);*/

    Command passLeft = new PassCommand(m_newShooter, m_hopper, m_intake, m_TurretSubsystem, null, m_activeHubTime);
    m_xBoxDriver.povLeft().whileTrue(passLeft);
    
    Command passRight = new PassCommand(m_newShooter, m_hopper, m_intake, m_TurretSubsystem, null, m_activeHubTime);
    m_xBoxDriver.povRight().whileTrue(passRight);

    Command intake = m_intake.intakeCommand();

    Command agitate = m_hopper.agitateCommand();
    m_xBoxOperator.rightBumper().whileTrue(agitate);

    Command outitate = m_hopper.inverseAgitateCommand();
    m_xBoxOperator.leftBumper().whileTrue(outitate);
    
    
    //Testing and Debugging Commands on Custom Controller
    Command doNothing = Commands.none();

    
    //m_CustomController.b().whileTrue(doNothing);
    //m_CustomController.y().whileTrue(doNothing);
    //m_CustomController.x().toggleOnTrue(doNothing);
    //m_CustomController.leftTrigger().whileTrue(doNothing);
    //m_CustomController.rightBumper().onTrue(Commands.runOnce(()->InitialAutonPathfind()));
    //m_CustomController.leftBumper().onTrue(doNothing);


   /* 
    m_xBoxDriver.a().whileTrue(doNothing);
    m_xBoxDriver.b().whileTrue(doNothing);
    m_xBoxDriver.x().whileTrue(doNothing);
    m_xBoxDriver.y().whileTrue(doNothing);
    m_xBoxDriver.rightBumper().whileTrue(doNothing);
    m_xBoxDriver.leftBumper().whileTrue(doNothing); */


    /*Command newShoot = m_newShooter.shootCommand();
    Command newHoodUp = m_newShooter.HoodUp();
    Command newHoodDown = m_newShooter.HoodDown();
    Command newVelocityShot = Commands.startEnd(  ()->m_newShooter.runShooterVelocity(ShooterConstants.shooterRPM),
                                           ()->m_newShooter.stopNewShooter(true),
                                           m_newShooter);
    
    m_FunnyController.leftBumper().whileTrue(intake);
    m_FunnyController.rightBumper().whileTrue(agitate);
    m_FunnyController.leftTrigger().whileTrue(shootManual);
    m_FunnyController.y().whileTrue(newHoodUp);
    m_FunnyController.a().whileTrue(foldOut);
    m_FunnyController.b().whileTrue(foldIn);
    m_FunnyController.x().whileTrue(newHoodDown);
    m_FunnyController.povRight().whileTrue(new MoveTurretCommand(m_TurretSubsystem,TurretConstants.turretSpeed));
    m_FunnyController.povLeft().whileTrue(new MoveTurretCommand(m_TurretSubsystem,-TurretConstants.turretSpeed));
    //m_FunnyController.povUp().whileTrue(m_TurretSubsystem.setMotortoZero());
    //m_FunnyController.povDown().onTrue(new TurretTrackCommand(m_TurretSubsystem, m_swerve,
    //                TurretStates.TRACK,m_turretLimelight));
    */
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
        m_hubMatch=true;
    }else {
        m_hubMatch=false;
    }
  } else {
    //RED Hub is Active
    m_activeHub="Red";
    if(currentAlliance=="Red"){
        m_hubMatch=true;
    }else {
        m_hubMatch=false;
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
      m_hubMatch=true;
    }else if(secondsRemaining>kTransitionEnd){
      m_activeHubPhase="Transition Shift";
      m_activeHubTime=secondsRemaining-kTransitionEnd;
      m_activeHub="Both";
      m_hubMatch=true;
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
      m_hubMatch=true;
    }
    
  }else {
    updateUndefinedHub();
  }
  

  }
  private void updateUndefinedHub(){
    m_activeHub="Undefined";
    m_hubMatch=false;
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
    //m_TurretSubsystem.showEncoderPositions();
    try{
            updateHubStatus();
      SmartDashboard.putBoolean("Our Hub Active",m_hubMatch);      
      SmartDashboard.putString("ActiveHub",m_activeHub);
      SmartDashboard.putString("Active Phase",m_activeHubPhase);
      SmartDashboard.putNumber("Active Phase Time",m_activeHubTime);
      SmartDashboard.putNumber("LimeLight Distance" , m_turretLimelight.getDistanceInverted());   
      SmartDashboard.putNumber("Hood Position is ", m_newShooter.getHoodPos());
      
    SmartDashboard.putBoolean("Is Climb Up ???",m_climb.isClimbUpLimit());
      SmartDashboard.putBoolean("Is Climb Down ???",m_climb.isClimbDownLimit());
    }
      catch(Exception e){}

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
  public void updateOdomfromLimeLight(){
   /*     if(m_backLimelight.isAnyTargetAvailable()){
      m_backLimelight.SetRobotOrientation(m_swerve.getPigeon2().getRotation2d().getDegrees(),0);
  
      Pose2d currentPose=m_backLimelight.GetPoseViaMegatag2();
      double currentTimeStamp=Utils.getCurrentTimeSeconds();

      System.out.println("update vision back " + currentPose.getX() + "/" + currentPose.getY());
      m_swerve.addVisionMeasurement(currentPose,currentTimeStamp);
    } 
      */

  }
  public void DisabledPeriodic(){
   updateOdomfromLimeLight();
/* 
    if(m_leftLimelight.isAnyTargetAvailable()){
      m_leftLimelight.SetRobotOrientation(m_swerve.getPigeon2().getRotation2d().getDegrees(),0);
  
      Pose2d currentPose=m_leftLimelight.GetPoseViaMegatag2();
      double currentTimeStamp=Utils.getCurrentTimeSeconds();

       System.out.println("update vision left "+ currentPose.getX() + "/" + currentPose.getY());
      m_swerve.addVisionMeasurement(currentPose,currentTimeStamp);
    }*/ 
    //m_LEDS.setLedState(LEDStates.DISABLED,false);
    //m_LEDS.activateLEDS();
  }
  
  public void AutoPeriodic(){
    /*if(m_backLimelight.isAnyTargetAvailable()){
      m_backLimelight.SetRobotOrientation(m_swerve.getPigeon2().getRotation2d().getDegrees(),0);
  
      Pose2d currentPose=m_backLimelight.GetPoseViaMegatag2();
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
    
  Pose2d startPose = new Pose2d(4,7.5, Rotation2d.fromDegrees(2));
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
    InitialAutonPathfind();
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
    updateOdomfromLimeLight();

    refreshSmartDashboard();
    updateLEDs();
    //m_ShotCalculator.clearShootingParameters();
    //ShotCalculator.ShootingParameters shootingInfo = m_ShotCalculator.getParameters(m_swerve);
    //System.out.println("Turret Angle: " + shootingInfo.turretAngle());
    //System.out.println("Turret Velocity:" + shootingInfo.turretVelocity());
    
    if(m_backLimelight.isAnyTargetAvailable()){
      m_backLimelight.SetRobotOrientation(m_swerve.getPigeon2().getRotation2d().getDegrees(),0);
  
      Pose2d currentPose=m_backLimelight.GetPoseViaMegatag2();
      double currentTimeStamp=Utils.getCurrentTimeSeconds();


      m_swerve.addVisionMeasurement(currentPose,currentTimeStamp);
    } 

    /*if(m_leftLimelight.isAnyTargetAvailable()){
      m_leftLimelight.SetRobotOrientation(m_swerve.getPigeon2().getRotation2d().getDegrees(),0);
  
      Pose2d currentPose=m_leftLimelight.GetPoseViaMegatag2();
      double currentTimeStamp=Utils.getCurrentTimeSeconds();


      m_swerve.addVisionMeasurement(currentPose,currentTimeStamp);
    } */
  }

  public void AllPeriodic(){
    m_Field2d.setRobotPose(m_swerve.getPose2d());
    SmartDashboard.putNumber("Match Time", DriverStation.getMatchTime()); //elastic
    SmartDashboard.putNumber("Voltage",RobotController.getBatteryVoltage()); //elastic
    refreshSmartDashboard();
  }

  public void homeRobot(){
    if(!hasHomed){
      m_intake.homeIntake(Constants.homeTimeOut);
      m_newShooter.hoodHome();
      m_climb.homeClimb(Constants.homeTimeOut);
      hasHomed = true;
    }
  }
 
  private void resetDefaultCommand(){
    //disabled drive
    m_swerve.setDefaultCommand(m_swerve.applyRequestDrive(m_xBoxDriver,translationAxis,strafeAxis,rotationAxis));
  }
}


  
  