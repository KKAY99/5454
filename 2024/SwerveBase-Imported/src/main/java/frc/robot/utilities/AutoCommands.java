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
import frc.robot.Constants;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.RotateArm;
import frc.robot.Constants.AutoConstants.AutonomousRoutines;
import frc.robot.Constants.AutoConstants.StartingLocations;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.commands.AutoDoNothingCommand;
import frc.robot.commands.MoveArmCommand;
import frc.robot.commands.ShootCommand;
import frc.robot.commands.IntakeToggleCommand;


public class AutoCommands {
  public Pose2d m_startingPose = new Pose2d();
  public Swerve m_swerve;

  private ShooterSubsystem m_shooter;
  private IntakeSubsystem m_intake;

  Command m_blueStartToLeftNote;
  Command m_blueLeftSpeakerToAmpNote;
  Command m_blueCenterSpeakerToCenterNote;
  Command m_blueSourceSpeakerToSourceNote;
  Command m_blueLeftNoteToCenterNote;
  Command m_blueCenterNoteToRightNote;
  Command m_blueRightNoteToNote5;
  Command m_sourceLongtoBlueShootLocation;
  Command m_blueAmpToSourceNote;
  Command m_blueAmpToLongAmpNote;
  Command m_blueStartToMoveOutPos;

  Command m_redStartToLeftNote;
  Command m_redLeftSpeakerToAmpNote;
  Command m_redCenterSpeakerToCenterNote;
  Command m_redSourceSpeakerToSourceNote;
  Command m_redLeftNoteToCenterNote;
  Command m_redCenterNoteToRightNote;
  Command m_redRightNoteToNote5;
  Command m_sourceLongtoRedShootLocation;
  Command m_redAmpToSourceNote;
  Command m_redAmpToLongAmpNote;
  Command m_redStartToMoveOutPos;

  Command m_shoot1;
  Command m_shoot2;
  Command m_shoot3;
  Command m_shoot4;
  Command m_shoot5;

  Command m_startIntake1;
  Command m_startIntake2;
  Command m_startIntake3;
  Command m_startIntake4;
  Command m_startIntake5;

  Command m_stopIntake1;
  Command m_stopIntake2;
  Command m_stopIntake3;
  Command m_stopIntake4;
  Command m_stopIntake5;

  public AutoCommands(Swerve swerveDrive,ShooterSubsystem shooter,IntakeSubsystem intake){
      newCommand();
      m_swerve=swerveDrive;
      m_shooter=shooter;
      m_intake=intake;
      CreateCommands();
  }

  public void CreateCommands(){
    Command m_blueStartToLeftNote=m_swerve.createPathCommand(CreateAutoPath(Constants.AutoConstants.blueCenterStartPos,
                                                          Constants.AutoConstants.locationBlueShortAmpNote));
    Command m_blueLeftSpeakerToAmpNote=m_swerve.createPathCommand(CreateAutoPath(Constants.AutoConstants.blueLeftSpeakerStartPos,
                                                          Constants.AutoConstants.locationBlueShortAmpNote));
    Command m_blueCenterSpeakerToCenterNote=m_swerve.createPathCommand(CreateAutoPath(Constants.AutoConstants.blueCenterStartPos,
                                                          Constants.AutoConstants.locationBlueShortCenterNote));
    Command m_blueSourceSpeakerToSourceNote=m_swerve.createPathCommand(CreateAutoPath(Constants.AutoConstants.blueRightSpeakerStartPos,
                                                          Constants.AutoConstants.locationBlueShortSourceNote));
    Command m_blueLeftNoteToCenterNote=m_swerve.createPathCommand(CreateAutoPath(Constants.AutoConstants.locationBlueShortAmpNote,
                                                          Constants.AutoConstants.locationBlueShortCenterNote));
    Command m_blueCenterNoteToRightNote=m_swerve.createPathCommand(CreateAutoPath(Constants.AutoConstants.locationBlueShortCenterNote,
                                                          Constants.AutoConstants.locationBlueShortSourceNote));
    Command m_blueRightNoteToNote5=m_swerve.createPathCommand(CreateAutoPath(Constants.AutoConstants.locationBlueShortSourceNote,
                                                          Constants.AutoConstants.locationLongSourceNote));
    Command m_blueAmpToSourceNote=m_swerve.createPathCommand(CreateAutoPath(Constants.AutoConstants.locationBlueShortAmpNote,
                                                          Constants.AutoConstants.locationBlueShortSourceNote));
    Command m_longSourcetoBlueShootLocation=m_swerve.createPathCommand(CreateAutoPath(Constants.AutoConstants.locationLongSourceNote,
                                                          Constants.AutoConstants.locationBlueLongRightWing));
    Command m_blueAmpToLongAmpNote=m_swerve.createPathCommand(CreateAutoPath(Constants.AutoConstants.locationBlueShortAmpNote,
                                                          Constants.AutoConstants.locationLongAmpNote));
    Command m_blueStartToMoveOutPos=m_swerve.createPathCommand(CreateAutoPath(Constants.AutoConstants.blueCenterStartPos,
                                                          Constants.AutoConstants.blueMoveOutOfBoundPos));

    Command m_redStartToLeftNote=m_swerve.createPathCommand(CreateAutoPath(Constants.AutoConstants.redCenterStartPos,
                                                          Constants.AutoConstants.locationRedShortAmpNote));
    Command m_redLeftSpeakerToAmpNote=m_swerve.createPathCommand(CreateAutoPath(Constants.AutoConstants.redRightSpeakerStartPos,
                                                          Constants.AutoConstants.locationRedShortAmpNote));
    Command m_redCenterSpeakerToCenterNote=m_swerve.createPathCommand(CreateAutoPath(Constants.AutoConstants.redCenterStartPos,
                                                          Constants.AutoConstants.locationRedShortCenterNote));
    Command m_redSourceSpeakerToSourceNote=m_swerve.createPathCommand(CreateAutoPath(Constants.AutoConstants.redLeftSpeakerStartPos,
                                                          Constants.AutoConstants.locationRedShortSourceNote));
    Command m_redLeftNoteToCenterNote=m_swerve.createPathCommand(CreateAutoPath(Constants.AutoConstants.locationRedShortAmpNote,
                                                          Constants.AutoConstants.locationRedShortCenterNote));
    Command m_redCenterNoteToRightNote=m_swerve.createPathCommand(CreateAutoPath(Constants.AutoConstants.locationRedShortCenterNote,
                                                          Constants.AutoConstants.locationRedShortSourceNote));
    Command m_redRightNoteToNote5=m_swerve.createPathCommand(CreateAutoPath(Constants.AutoConstants.locationRedShortSourceNote,
                                                          Constants.AutoConstants.locationLongSourceNote));
    Command m_longSourcetoRedShootLocation=m_swerve.createPathCommand(CreateAutoPath(Constants.AutoConstants.locationLongSourceNote,
                                                          Constants.AutoConstants.locationRedLongRightWing));
    Command m_redAmpToSourceNote=m_swerve.createPathCommand(CreateAutoPath(Constants.AutoConstants.locationRedShortAmpNote,
                                                          Constants.AutoConstants.locationRedShortSourceNote));
    Command m_redAmpToLongAmpNote=m_swerve.createPathCommand(CreateAutoPath(Constants.AutoConstants.locationRedShortAmpNote,
                                                          Constants.AutoConstants.locationLongAmpNote));
    Command m_redStartToMoveOutPos=m_swerve.createPathCommand(CreateAutoPath(Constants.AutoConstants.redCenterStartPos,
                                                          Constants.AutoConstants.redMoveOutOfBoundPos));
  
    Command m_shoot1=new ShootCommand(m_shooter,null,Constants.ShooterConstants.autoShooterSpeed);
    Command m_shoot2=new ShootCommand(m_shooter,null,Constants.ShooterConstants.autoShooterSpeed);
    Command m_shoot3=new ShootCommand(m_shooter,null,Constants.ShooterConstants.autoShooterSpeed);
    Command m_shoot4=new ShootCommand(m_shooter,null,Constants.ShooterConstants.autoShooterSpeed);
    Command m_shoot5=new ShootCommand(m_shooter,null,Constants.ShooterConstants.autoShooterSpeed);

    Command m_startIntake1=new IntakeToggleCommand(m_intake,true);
    Command m_startIntake2=new IntakeToggleCommand(m_intake,true);
    Command m_startIntake3=new IntakeToggleCommand(m_intake,true);
    Command m_startIntake4=new IntakeToggleCommand(m_intake,true);
    Command m_startIntake5=new IntakeToggleCommand(m_intake,true);

    Command m_stopIntake1=new IntakeToggleCommand(m_intake,false);
    Command m_stopIntake2=new IntakeToggleCommand(m_intake,false);
    Command m_stopIntake3=new IntakeToggleCommand(m_intake,false);
    Command m_stopIntake4=new IntakeToggleCommand(m_intake,false);
    Command m_stopIntake5=new IntakeToggleCommand(m_intake,false);
  }

  private void newCommand(){
    m_startingPose= new Pose2d(0,0,new Rotation2d(0));
  }

  private Pose2d createPose(double x, double y, double rotate){
    return new Pose2d(x,y,new Rotation2d(rotate));
  }

  public Command createAutoCommand(AutoConstants.StartingLocations startinglocation,String routine,double startDelay,Alliance currentAlliance){
    Command autoRoutine = new AutoDoNothingCommand();
    newCommand();
    m_startingPose=getStartingPose(startinglocation,currentAlliance);

    switch(routine){
      case AutoConstants.autoMode0:
        autoRoutine=new AutoDoNothingCommand();
      break;
      case AutoConstants.autoMode1:
        if(currentAlliance==Alliance.Red){    
          redScore1MoveOut();     
        }else{
          blueScore1MoveOut();
        }
      break;
      case AutoConstants.autoMode2:
        if(currentAlliance==Alliance.Red){ 
          redScoreAmp2();
        }else{
          blueScoreAmp2();
        }
      break;
      case AutoConstants.autoMode3:
        if(currentAlliance==Alliance.Red){    
          redScoreCenter2();   
        }else{
          blueScoreCenter2();
        }
      break;
      case AutoConstants.autoMode4:
        if(currentAlliance==Alliance.Red){
          redScoreSource2();     
        }else{
          blueScoreSource2();
        }
      break;
      case AutoConstants.autoMode5:
        if(currentAlliance==Alliance.Red){
          redScoreAmpSource3();     
        }else{
          blueScoreAmpSource3();
        }
      break;
      case AutoConstants.autoMode6:
        if(currentAlliance==Alliance.Red){ 
          redScoreAmpPauseSource3();      
        }else{
          blueScoreAmpPauseSource3();   
        }
      break;
      case AutoConstants.autoMode7:
        if(currentAlliance==Alliance.Red){  
          redScoreAmpLongAmp3();     
        }else{
          blueScoreAmpLongAmp3();  
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
                                                                 m_stopIntake2,m_shoot3,m_startIntake3,m_redCenterNoteToRightNote,m_shoot4,m_stopIntake3);

      return score4Note;
  }

  private Command blueAutoScore4Notes(){
      SequentialCommandGroup score4Note=new SequentialCommandGroup(m_shoot1,m_startIntake1,m_blueStartToLeftNote,m_stopIntake1,m_shoot2,m_startIntake2,m_blueLeftNoteToCenterNote,
                                                                m_stopIntake2,m_shoot3,m_startIntake3,m_blueCenterNoteToRightNote,m_startIntake3,m_shoot4);

      return score4Note;
  }

  private Command redAutoScore5NotesRight(){
      SequentialCommandGroup score4Note=new SequentialCommandGroup(m_shoot1,m_redStartToLeftNote,m_shoot2,m_redLeftNoteToCenterNote,
                                                                 m_shoot3,m_redCenterNoteToRightNote,m_shoot4,m_redRightNoteToNote5,
                                                                 m_sourceLongtoRedShootLocation,m_shoot5);

      return score4Note;
  }
}


