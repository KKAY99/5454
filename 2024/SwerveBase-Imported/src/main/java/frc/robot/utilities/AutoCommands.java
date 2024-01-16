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
import frc.robot.subsystems.RotateArmSubsystem;
import frc.robot.subsystems.Swerve;
import frc.robot.Constants;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.RotateArm;
import frc.robot.Constants.AutoConstants.AutonomousRoutines;
import frc.robot.Constants.AutoConstants.StartingLocations;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.commands.AutoDoNothingCommand;
import frc.robot.commands.MoveArmCommand;


public class AutoCommands {
  public Pose2d m_startingPose = new Pose2d();
  public Swerve m_swerve;
  public RotateArmSubsystem m_rotateArm;

  public AutoCommands(Swerve swerveDrive,RotateArmSubsystem arm){
      newCommand();
      m_swerve=swerveDrive;
      m_rotateArm=arm;
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
        }else{
        }
      break;
      case AutoConstants.autoMode2:
        if(currentAlliance==Alliance.Red){
          autoRoutine=redAutoScore5NotesRight();         
        }else{
        }
      break;
      case AutoConstants.autoMode3:
        if(currentAlliance==Alliance.Red){       
        }else{
        }
      break;
      case AutoConstants.autoMode4:
        if(currentAlliance==Alliance.Red){       
        }else{
        }
      break;
      case AutoConstants.autoMode5:
        if(currentAlliance==Alliance.Red){       
        }else{
        }
      break;
      case AutoConstants.autoMode6:
        if(currentAlliance==Alliance.Red){       
        }else{
        }
      break;
      case AutoConstants.autoMode7:
        if(currentAlliance==Alliance.Red){       
        }else{
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

  private Command redAutoScore4Notes(){
      SequentialCommandGroup mockScore1=new SequentialCommandGroup(new MoveArmCommand(m_rotateArm,Constants.RotateArm.armSpeed),
                                            new MoveArmCommand(m_rotateArm,-Constants.RotateArm.armSpeed));

      SequentialCommandGroup mockScore2=new SequentialCommandGroup(new MoveArmCommand(m_rotateArm,Constants.RotateArm.armSpeed),
                                            new MoveArmCommand(m_rotateArm,-Constants.RotateArm.armSpeed));

      SequentialCommandGroup mockScore3=new SequentialCommandGroup(new MoveArmCommand(m_rotateArm,Constants.RotateArm.armSpeed),
                                            new MoveArmCommand(m_rotateArm,-Constants.RotateArm.armSpeed));

      SequentialCommandGroup mockScore4=new SequentialCommandGroup(new MoveArmCommand(m_rotateArm,Constants.RotateArm.armSpeed),
                                            new MoveArmCommand(m_rotateArm,-Constants.RotateArm.armSpeed));
                                            

      Command startToLeftNote=m_swerve.createPathCommand(CreateAutoPath(Constants.AutoConstants.redCenterStartPos,
                                                              Constants.AutoConstants.locationRedShortAmpNote));

      Command leftNoteToCenterNote=m_swerve.createPathCommand(CreateAutoPath(Constants.AutoConstants.locationRedShortAmpNote,
                                                              Constants.AutoConstants.locationRedShortCenterNote));

      Command centerNoteToRightNote=m_swerve.createPathCommand(CreateAutoPath(Constants.AutoConstants.locationRedShortCenterNote,
                                                              Constants.AutoConstants.locationRedShortSourceNote));

      SequentialCommandGroup score4Note=new SequentialCommandGroup(mockScore1,startToLeftNote,mockScore2,leftNoteToCenterNote,
                                                                 mockScore3,centerNoteToRightNote,mockScore4);

      return score4Note;
  }

  private Command blueAutoScore4Notes(){
      SequentialCommandGroup mockScore1=new SequentialCommandGroup(new MoveArmCommand(m_rotateArm,Constants.RotateArm.armSpeed),
                                            new MoveArmCommand(m_rotateArm,-Constants.RotateArm.armSpeed));

      SequentialCommandGroup mockScore2=new SequentialCommandGroup(new MoveArmCommand(m_rotateArm,Constants.RotateArm.armSpeed),
                                            new MoveArmCommand(m_rotateArm,-Constants.RotateArm.armSpeed));

      SequentialCommandGroup mockScore3=new SequentialCommandGroup(new MoveArmCommand(m_rotateArm,Constants.RotateArm.armSpeed),
                                            new MoveArmCommand(m_rotateArm,-Constants.RotateArm.armSpeed));

      SequentialCommandGroup mockScore4=new SequentialCommandGroup(new MoveArmCommand(m_rotateArm,Constants.RotateArm.armSpeed),
                                            new MoveArmCommand(m_rotateArm,-Constants.RotateArm.armSpeed));
                                            

      Command startToLeftNote=m_swerve.createPathCommand(CreateAutoPath(Constants.AutoConstants.blueCenterStartPos,
                                                              Constants.AutoConstants.locationBlueShortAmpNote));

      Command leftNoteToCenterNote=m_swerve.createPathCommand(CreateAutoPath(Constants.AutoConstants.locationBlueShortAmpNote,
                                                              Constants.AutoConstants.locationBlueShortCenterNote));

      Command centerNoteToRightNote=m_swerve.createPathCommand(CreateAutoPath(Constants.AutoConstants.locationBlueShortCenterNote,
                                                              Constants.AutoConstants.locationBlueShortSourceNote));

      SequentialCommandGroup score4Note=new SequentialCommandGroup(mockScore1,startToLeftNote,mockScore2,leftNoteToCenterNote,
                                                                 mockScore3,centerNoteToRightNote,mockScore4);

      return score4Note;
  }

  private Command redAutoScore5NotesRight(){
      SequentialCommandGroup mockScore1=new SequentialCommandGroup(new MoveArmCommand(m_rotateArm,Constants.RotateArm.armSpeed),
                                            new MoveArmCommand(m_rotateArm,-Constants.RotateArm.armSpeed));

      SequentialCommandGroup mockScore2=new SequentialCommandGroup(new MoveArmCommand(m_rotateArm,Constants.RotateArm.armSpeed),
                                            new MoveArmCommand(m_rotateArm,-Constants.RotateArm.armSpeed));

      SequentialCommandGroup mockScore3=new SequentialCommandGroup(new MoveArmCommand(m_rotateArm,Constants.RotateArm.armSpeed),
                                            new MoveArmCommand(m_rotateArm,-Constants.RotateArm.armSpeed));

      SequentialCommandGroup mockScore4=new SequentialCommandGroup(new MoveArmCommand(m_rotateArm,Constants.RotateArm.armSpeed),
                                            new MoveArmCommand(m_rotateArm,-Constants.RotateArm.armSpeed));

      SequentialCommandGroup mockScore5=new SequentialCommandGroup(new MoveArmCommand(m_rotateArm,Constants.RotateArm.armSpeed),
                                            new MoveArmCommand(m_rotateArm,-Constants.RotateArm.armSpeed));
                                            

      Command startToLeftNote=m_swerve.createPathCommand(CreateAutoPath(Constants.AutoConstants.redCenterStartPos,
                                                              Constants.AutoConstants.locationRedShortAmpNote));

      Command leftNoteToCenterNote=m_swerve.createPathCommand(CreateAutoPath(Constants.AutoConstants.locationRedShortAmpNote,
                                                              Constants.AutoConstants.locationRedShortCenterNote));

      Command centerNoteToRightNote=m_swerve.createPathCommand(CreateAutoPath(Constants.AutoConstants.locationRedShortCenterNote,
                                                              Constants.AutoConstants.locationRedShortSourceNote));

      Command rightNoteToNote5=m_swerve.createPathCommand(CreateAutoPath(Constants.AutoConstants.locationRedShortSourceNote,
                                                              Constants.AutoConstants.locationLongSourceNote));

      Command note5toShootLocation=m_swerve.createPathCommand(CreateAutoPath(Constants.AutoConstants.locationLongSourceNote,
                                                              Constants.AutoConstants.locationRedLongRightWing));

      SequentialCommandGroup score4Note=new SequentialCommandGroup(mockScore1,startToLeftNote,mockScore2,leftNoteToCenterNote,
                                                                 mockScore3,centerNoteToRightNote,mockScore4,rightNoteToNote5,
                                                                 note5toShootLocation,mockScore5);

      return score4Note;
  }
}


