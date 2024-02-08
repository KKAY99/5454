package frc.robot.utilities;
import java.util.List;

import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.RotateArmSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.Constants;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.RotateArm;
import frc.robot.Constants.AutoConstants.AutonomousRoutines;
import frc.robot.Constants.AutoConstants.StartingLocations;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.commands.AutoDoNothingCommand;
import frc.robot.commands.MoveArmCommand;
import frc.robot.commands.ShootCommand;
import frc.robot.commands.TurretCommand;
import frc.robot.commands.IntakeToggleCommand;


public class AutoCommands {
  public Pose2d m_startingPose = new Pose2d();
  public Swerve m_swerve;

  private ShooterSubsystem m_shooter;
  private IntakeSubsystem m_intake;
  private Limelight m_limeLight;
  private TurretSubsystem m_turret;

  private Command m_blueStartToLeftNote;
  private Command m_blueLeftSpeakerToAmpNote;
  private Command m_blueCenterSpeakerToCenterNote;
  private Command m_blueSourceSpeakerToSourceNote;
  private Command m_blueLeftNoteToCenterNote;
  private Command m_blueCenterNoteToRightNote;
  private Command m_blueRightNoteToNote5;
  private Command m_sourceLongtoBlueShootLocation;
  private Command m_blueAmpToSourceNote;
  private Command m_blueAmpToLongAmpNote;
  private Command m_blueStartToMoveOutPos;

  private Command m_redStartToLeftNote;
  private Command m_redLeftSpeakerToAmpNote;
  private Command m_redCenterSpeakerToCenterNote;
  private Command m_redSourceSpeakerToSourceNote;
  private Command m_redLeftNoteToCenterNote;
  private Command m_redCenterNoteToRightNote;
  private Command m_redRightNoteToNote5;
  private Command m_sourceLongtoRedShootLocation;
  private Command m_redAmpToSourceNote;
  private Command m_redAmpToLongAmpNote;
  private Command m_redStartToMoveOutPos;

  private Command m_shoot1;
  private Command m_shoot2;
  private Command m_shoot3;
  private Command m_shoot4;
  private Command m_shoot5;

  private Command m_startIntake1;
  private Command m_startIntake2;
  private Command m_startIntake3;
  private Command m_startIntake4;
  private Command m_startIntake5;

  private Command m_stopIntake1;
  private Command m_stopIntake2;
  private Command m_stopIntake3;
  private Command m_stopIntake4;
  private Command m_stopIntake5;

  public AutoCommands(Swerve swerveDrive,ShooterSubsystem shooter,IntakeSubsystem intake,TurretSubsystem turret,Limelight limelight){
      newCommand();
      m_swerve=swerveDrive;
      m_shooter=shooter;
      m_intake=intake;
      m_turret=turret;
      m_limeLight=limelight;
      CreateCommands();
  }

  public void CreateCommands(){
    m_blueStartToLeftNote=m_swerve.createPathCommand(CreateAutoPath(m_swerve.getPose(),
                                                          Constants.AutoConstants.locationBlueShortAmpNote));
    m_blueLeftSpeakerToAmpNote=m_swerve.createPathCommand(CreateAutoPath(m_swerve.getPose(),
                                                          Constants.AutoConstants.locationBlueShortAmpNote));
    m_blueCenterSpeakerToCenterNote=m_swerve.createPathCommand(CreateAutoPath(m_swerve.getPose(),
                                                          Constants.AutoConstants.locationBlueShortCenterNote));
    m_blueSourceSpeakerToSourceNote=m_swerve.createPathCommand(CreateAutoPath(m_swerve.getPose(),
                                                          Constants.AutoConstants.locationBlueShortSourceNote));
    m_blueLeftNoteToCenterNote=m_swerve.createPathCommand(CreateAutoPath(Constants.AutoConstants.locationBlueShortAmpNote,
                                                          Constants.AutoConstants.locationBlueShortCenterNote));
    m_blueCenterNoteToRightNote=m_swerve.createPathCommand(CreateAutoPath(Constants.AutoConstants.locationBlueShortCenterNote,
                                                          Constants.AutoConstants.locationBlueShortSourceNote));
    m_blueRightNoteToNote5=m_swerve.createPathCommand(CreateAutoPath(Constants.AutoConstants.locationBlueShortSourceNote,
                                                          Constants.AutoConstants.locationLongSourceNote));
    m_blueAmpToSourceNote=m_swerve.createPathCommand(CreateAutoPath(Constants.AutoConstants.locationBlueShortAmpNote,
                                                          Constants.AutoConstants.locationBlueShortSourceNote));
    m_sourceLongtoBlueShootLocation=m_swerve.createPathCommand(CreateAutoPath(Constants.AutoConstants.locationLongSourceNote,
                                                          Constants.AutoConstants.locationBlueLongRightWing));
    m_blueAmpToLongAmpNote=m_swerve.createPathCommand(CreateAutoPath(Constants.AutoConstants.locationBlueShortAmpNote,
                                                          Constants.AutoConstants.locationLongAmpNote));
    m_blueStartToMoveOutPos=m_swerve.createPathCommand(CreateAutoPath(m_swerve.getPose(),
                                                          Constants.AutoConstants.blueMoveOutOfBoundPos));

    m_redStartToLeftNote=m_swerve.createPathCommand(CreateAutoPath(m_swerve.getPose(),
                                                          Constants.AutoConstants.locationRedShortAmpNote));
    m_redLeftSpeakerToAmpNote=m_swerve.createPathCommand(CreateAutoPath(Constants.AutoConstants.redRightSpeakerStartPos,
                                                          Constants.AutoConstants.locationRedShortAmpNote));
    m_redCenterSpeakerToCenterNote=m_swerve.createPathCommand(CreateAutoPath(m_swerve.getPose(),
                                                          Constants.AutoConstants.locationRedShortCenterNote));
    m_redSourceSpeakerToSourceNote=m_swerve.createPathCommand(CreateAutoPath(m_swerve.getPose(),
                                                          Constants.AutoConstants.locationRedShortSourceNote));
    m_redLeftNoteToCenterNote=m_swerve.createPathCommand(CreateAutoPath(Constants.AutoConstants.locationRedShortAmpNote,
                                                          Constants.AutoConstants.locationRedShortCenterNote));
    m_redCenterNoteToRightNote=m_swerve.createPathCommand(CreateAutoPath(Constants.AutoConstants.locationRedShortCenterNote,
                                                          Constants.AutoConstants.locationRedShortSourceNote));
    m_redRightNoteToNote5=m_swerve.createPathCommand(CreateAutoPath(Constants.AutoConstants.locationRedShortSourceNote,
                                                          Constants.AutoConstants.locationLongSourceNote));
    m_sourceLongtoRedShootLocation=m_swerve.createPathCommand(CreateAutoPath(Constants.AutoConstants.locationLongSourceNote,
                                                          Constants.AutoConstants.locationRedLongRightWing));
    m_redAmpToSourceNote=m_swerve.createPathCommand(CreateAutoPath(Constants.AutoConstants.locationRedShortAmpNote,
                                                          Constants.AutoConstants.locationRedShortSourceNote));
    m_redAmpToLongAmpNote=m_swerve.createPathCommand(CreateAutoPath(Constants.AutoConstants.locationRedShortAmpNote,
                                                          Constants.AutoConstants.locationLongAmpNote));
    m_redStartToMoveOutPos=m_swerve.createPathCommand(CreateAutoPath(m_swerve.getPose(),
                                                          Constants.AutoConstants.redMoveOutOfBoundPos));
  
    m_shoot1=new ShootCommand(m_shooter,Constants.ShooterConstants.autoShooterSpeed);
    m_shoot2=new ShootCommand(m_shooter,Constants.ShooterConstants.autoShooterSpeed);
    m_shoot3=new ShootCommand(m_shooter,Constants.ShooterConstants.autoShooterSpeed);
    m_shoot4=new ShootCommand(m_shooter,Constants.ShooterConstants.autoShooterSpeed);
    m_shoot5=new ShootCommand(m_shooter,Constants.ShooterConstants.autoShooterSpeed);

    m_startIntake1=new IntakeToggleCommand(m_intake);
    m_startIntake2=new IntakeToggleCommand(m_intake);
    m_startIntake3=new IntakeToggleCommand(m_intake);
    m_startIntake4=new IntakeToggleCommand(m_intake);
    m_startIntake5=new IntakeToggleCommand(m_intake);

    m_stopIntake1=new IntakeToggleCommand(m_intake);
    m_stopIntake2=new IntakeToggleCommand(m_intake);
    m_stopIntake3=new IntakeToggleCommand(m_intake);
    m_stopIntake4=new IntakeToggleCommand(m_intake);
    m_stopIntake5=new IntakeToggleCommand(m_intake);
  }

  private void newCommand(){
    m_startingPose= new Pose2d(0,0,new Rotation2d(0));
  }

  private Pose2d createPose(double x, double y, double rotate){
    return new Pose2d(x,y,new Rotation2d(rotate));
  }

  public Command createAutoCommand(AutoConstants.StartingLocations startinglocation,String routine,double startDelay,Alliance currentAlliance){
    Command autoRoutine = null;
    
    m_startingPose=getStartingPose(startinglocation,currentAlliance);

    switch(routine){
      case AutoConstants.autoMode0:
        autoRoutine=new AutoDoNothingCommand();
      break;
      case AutoConstants.autoMode1:
        if(currentAlliance==Alliance.Red){    
          autoRoutine=redScore1MoveOut();     
        }else{
          autoRoutine=blueScore1MoveOut();
        }
      break;
      case AutoConstants.autoMode2:
        if(currentAlliance==Alliance.Red){ 
          autoRoutine=redScoreAmp2();
        }else{
          autoRoutine=blueScoreAmp2();
        }
      break;
      case AutoConstants.autoMode3:
        if(currentAlliance==Alliance.Red){    
          autoRoutine=redScoreCenter2();   
        }else{
          autoRoutine=blueScoreCenter2();
        }
      break;
      case AutoConstants.autoMode4:
        if(currentAlliance==Alliance.Red){
          autoRoutine=redScoreSource2();     
        }else{
          autoRoutine=blueScoreSource2();
        }
      break;
      case AutoConstants.autoMode5:
        if(currentAlliance==Alliance.Red){
          autoRoutine=redScoreAmpSource3();     
        }else{
          autoRoutine=blueScoreAmpSource3();
        }
      break;
      case AutoConstants.autoMode6:
        if(currentAlliance==Alliance.Red){ 
          autoRoutine=redScoreAmpPauseSource3();      
        }else{
          autoRoutine=blueScoreAmpPauseSource3();   
        }
      break;
      case AutoConstants.autoMode7:
        if(currentAlliance==Alliance.Red){  
          autoRoutine=redScoreAmpLongAmp3();     
        }else{
          autoRoutine=blueScoreAmpLongAmp3();  
        }
      break;
      case AutoConstants.autoMode8:
        if(currentAlliance==Alliance.Red){       
        }else{
        }
      break;
      case AutoConstants.autoMode9:
        if(currentAlliance==Alliance.Red){       
        }else{
        }
      break;
      case AutoConstants.autoMode10:
        if(currentAlliance==Alliance.Red){
          autoRoutine=redAutoScore4Notes();      
        }else{
          autoRoutine=blueAutoScore4Notes(); 
        }
      break;
      case AutoConstants.autoMode11:
        if(currentAlliance==Alliance.Red){ 
          autoRoutine=redAutoScore4Notes();       
        }else{
          autoRoutine=blueAutoScore4Notes();
        }
      break;
    }
    return autoRoutine;
 }

  public Pose2d getStartingPose(AutoConstants.StartingLocations location,Alliance currentAlliance){
    Pose2d returnPose=new Pose2d();
      switch(location){
        case LEFTAMP:
        if(currentAlliance==Alliance.Red){   
          returnPose=AutoConstants.redLeftAmpStartPos;     
        }else{
          returnPose=AutoConstants.blueLeftAmpStartPos;
        }
        break;
        case LEFTSPEAKER:
        if(currentAlliance==Alliance.Red){
          returnPose=AutoConstants.redLeftSpeakerStartPos;        
        }else{
          returnPose=AutoConstants.blueLeftSpeakerStartPos; 
        }
        break;
        case CENTER1:
        if(currentAlliance==Alliance.Red){
          returnPose=AutoConstants.redCenterStartPos;      
        }else{
          returnPose=AutoConstants.blueCenterStartPos;   
        }
        break;
        case RIGHTSPEAKER:
        if(currentAlliance==Alliance.Red){ 
          returnPose=AutoConstants.redRightSpeakerStartPos;        
        }else{
          returnPose=AutoConstants.blueRightSpeakerStartPos;  
        }
        break;
      }
    return returnPose;
  }

  public PathPlannerPath CreateAutoPath(Pose2d startPose,Pose2d desiredPose){
      List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(
        startPose,
        desiredPose
      );

      PathPlannerPath newPath = new PathPlannerPath(
            bezierPoints,
            new PathConstraints(3.0, 3.0, 2 * Math.PI, 4 * Math.PI), // The constraints for this path. If using a differential drivetrain, the angular constraints have no effect.
            new GoalEndState(0.0, Rotation2d.fromDegrees(0)) // Goal end state. You can set a holonomic rotation here. If using a differential drivetrain, the rotation will have no effect.
      );

      return newPath;
  }

  private Command blueScore1MoveOut(){
    SequentialCommandGroup score2=new SequentialCommandGroup(m_shoot1,m_blueStartToMoveOutPos);

    return score2;
  }

  private Command redScore1MoveOut(){
    SequentialCommandGroup score2=new SequentialCommandGroup(m_shoot1,m_redStartToMoveOutPos);

    return score2;
  }

  private Command blueScoreAmp2(){
    SequentialCommandGroup scoreCenter2=new SequentialCommandGroup(m_shoot1,m_startIntake1,m_blueLeftSpeakerToAmpNote,m_stopIntake1,
                                                                  m_shoot2);

    return scoreCenter2;
  }

  private Command redScoreAmp2(){
    SequentialCommandGroup scoreCenter2=new SequentialCommandGroup(m_shoot1,m_startIntake1,m_redLeftSpeakerToAmpNote,m_stopIntake1,
                                                                  m_shoot2);

    return scoreCenter2;
  }

  private Command blueScoreCenter2(){
    SequentialCommandGroup score2=new SequentialCommandGroup(m_shoot1,m_startIntake1,m_blueCenterSpeakerToCenterNote,m_stopIntake1,
                                                            m_shoot2);

    return score2;
  }

  private Command redScoreCenter2(){
    SequentialCommandGroup score2=new SequentialCommandGroup(m_shoot1,m_startIntake1,m_redCenterSpeakerToCenterNote,m_stopIntake1,
                                                            m_shoot2);

    return score2;
  }

  private Command redScoreSource2(){
    SequentialCommandGroup score2=new SequentialCommandGroup(m_shoot1,m_startIntake1,m_redSourceSpeakerToSourceNote,m_stopIntake1,
                                                            m_shoot2);

    return score2;
  }

  private Command blueScoreSource2(){
    SequentialCommandGroup score2=new SequentialCommandGroup(m_shoot1,m_startIntake1,m_blueSourceSpeakerToSourceNote,m_stopIntake1,
                                                            m_shoot2);

    return score2;
  }

  private Command redScoreAmpSource3(){
    SequentialCommandGroup score3=new SequentialCommandGroup(m_shoot1,m_startIntake1,m_redLeftSpeakerToAmpNote,m_stopIntake1,
                                                            m_shoot2,m_startIntake2,m_redAmpToSourceNote,m_stopIntake2,m_shoot3);

    return score3;
  }

  private Command blueScoreAmpSource3(){
    SequentialCommandGroup score3=new SequentialCommandGroup(m_shoot1,m_startIntake1,m_blueLeftSpeakerToAmpNote,m_stopIntake1,
                                                            m_shoot2,m_startIntake2,m_blueAmpToSourceNote,m_stopIntake2,m_shoot3);
    return score3;
  }

  private Command redScoreAmpPauseSource3(){
    SequentialCommandGroup score3=new SequentialCommandGroup(m_shoot1,m_startIntake1,m_redLeftSpeakerToAmpNote,m_stopIntake1,
                                                            m_shoot2,m_startIntake2,m_redAmpToSourceNote,m_stopIntake2,m_shoot3);

    return score3;
  }

  private Command blueScoreAmpPauseSource3(){
    SequentialCommandGroup score3=new SequentialCommandGroup(m_shoot1,m_startIntake1,m_blueLeftSpeakerToAmpNote,m_stopIntake1,
                                                            m_shoot2,m_startIntake2,m_blueAmpToSourceNote,m_stopIntake2,m_shoot3);
    return score3;
  }

  private Command redScoreAmpLongAmp3(){
    SequentialCommandGroup score3=new SequentialCommandGroup(m_shoot1,m_startIntake1,m_redLeftSpeakerToAmpNote,m_stopIntake1,
                                                            m_shoot2,m_startIntake2,m_redAmpToLongAmpNote,m_stopIntake2,m_shoot3);

    return score3;
  }

  private Command blueScoreAmpLongAmp3(){
    SequentialCommandGroup score3=new SequentialCommandGroup(m_shoot1,m_startIntake1,m_blueLeftSpeakerToAmpNote,m_stopIntake1,
                                                            m_shoot2,m_startIntake2,m_blueAmpToLongAmpNote,m_stopIntake2,m_shoot3);
    return score3;
  }

  private Command redAutoScore4Notes(){                             
      SequentialCommandGroup score4Note=new SequentialCommandGroup(m_shoot1,m_startIntake1,m_redStartToLeftNote,m_stopIntake1,m_shoot2,m_startIntake2,m_redLeftNoteToCenterNote,
                                                                 m_stopIntake2,m_shoot3,m_startIntake3,m_redCenterNoteToRightNote,m_stopIntake3,m_shoot4);
      
      return score4Note;
  }

  private Command blueAutoScore4Notes(){
     SequentialCommandGroup score4Note=new SequentialCommandGroup(m_shoot1,m_startIntake1,m_blueStartToLeftNote,m_stopIntake1,m_shoot2,m_startIntake2,m_blueLeftNoteToCenterNote,
                                                               m_stopIntake2,m_shoot3,m_startIntake3,m_blueCenterNoteToRightNote,m_stopIntake3,m_shoot4);

     return score4Note;
  }

  private Command redAutoScore5NotesRight(){
      SequentialCommandGroup score4Note=new SequentialCommandGroup(m_shoot1,m_redStartToLeftNote,m_shoot2,m_redLeftNoteToCenterNote,
                                                                 m_shoot3,m_redCenterNoteToRightNote,m_shoot4,m_redRightNoteToNote5,
                                                                 m_sourceLongtoRedShootLocation,m_shoot5);

      return score4Note;
  }
}


