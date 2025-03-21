package frc.robot.utilities;

import java.util.ArrayList;
import java.util.List;
import java.util.function.DoubleSupplier;
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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.Supplier;

public class AutoPlanner {
    private PathConstraints pathConstraints;

    //Default Constraint Values
    private double m_velocityMPS=0.7;
    private double m_maxAccelMPS=0.7;
    private double m_angularVelocityMPS=6;
    private double m_angularMaxAccelMPS=6;
    
    public AutoPlanner(){
      pathConstraints=new PathConstraints(m_velocityMPS,m_maxAccelMPS,m_angularVelocityMPS,m_angularMaxAccelMPS);
    }
 
    public PathPlannerPath CreateOdomLineUpPath(Pose2d startPose,Pose2d targetPose,Pose2d targetLineupPose){

    List<Waypoint> waypoints= PathPlannerPath.waypointsFromPoses(
        startPose,targetLineupPose,targetPose
    );

    PathPlannerPath newPath=new PathPlannerPath(
    waypoints, 
    pathConstraints,
    new IdealStartingState(0,startPose.getRotation()),
    new GoalEndState(0,targetPose.getRotation()));
    
    return newPath;
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
