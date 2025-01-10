package frc.robot.utilities;

import java.util.ArrayList;
import java.util.List;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.IdealStartingState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPoint;
import com.pathplanner.lib.path.Waypoint;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;

public class AutoPlanner {
    private PathConstraints pathConstraints;

    //Default Constraint Values
    private double m_velocityMPS=2.0;
    private double m_maxAccelMPS=2.0;
    private double m_angularVelocityMPS=11.5;
    private double m_angularMaxAccelMPS=11.5;
    
    public AutoPlanner(){
      pathConstraints=new PathConstraints(m_velocityMPS,m_maxAccelMPS,m_angularVelocityMPS,m_angularMaxAccelMPS);
    }

    public PathPlannerPath CreateAutoPath(double endPosRot,double endVelocity,Pose2d... poses){
    List<PathPoint> pathPointArray=new ArrayList<PathPoint>();

    for(Pose2d pose:poses){
      if(pose!=null){
        pathPointArray.add(new PathPoint(new Translation2d(pose.getX(),pose.getY())));
      }
    }

    PathPlannerPath newPath = PathPlannerPath.fromPathPoints(
        pathPointArray,pathConstraints, 
        new GoalEndState(endVelocity, Rotation2d.fromDegrees(endPosRot))
    );

    if(DriverStation.getAlliance().get()==Alliance.Blue){
      return newPath;
    }else{
      return newPath.flipPath();
    }
  }

  public Command CreatePathfindingPath(double endVelocity,Pose2d targetPose){
    if(DriverStation.getAlliance().get()==Alliance.Blue){
      return AutoBuilder.pathfindToPose(targetPose,pathConstraints);
    }else{
      return AutoBuilder.pathfindToPoseFlipped(targetPose,pathConstraints);
    }
  }
  

  public void SetPathConstraints(double velocityMPS,double maxAccelMPS,double angularVelocityMPS,double angularMaxAccelMPS){
    m_velocityMPS=velocityMPS;
    m_maxAccelMPS=maxAccelMPS;
    m_angularVelocityMPS=angularVelocityMPS;
    m_angularMaxAccelMPS=angularMaxAccelMPS;
  }
}
