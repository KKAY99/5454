package frc.robot.utilities;

import java.util.ArrayList;
import java.util.List;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.IdealStartingState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class AutoPlanner {
    
    public AutoPlanner(){

    }

    public PathPlannerPath CreateAutoPath(Pose2d... poses){
    ArrayList<Pose2d> poseArray=new ArrayList<>();

    for(Pose2d pose:poses){
      if(pose!=null){
        poseArray.add(pose);
      }
    }

    List<Waypoint> bezierPoints = PathPlannerPath.waypointsFromPoses(
        poseArray);

    PathPlannerPath newPath = new PathPlannerPath(
        bezierPoints,
        new PathConstraints(3.0, 3.0, 2 * Math.PI, 4 * Math.PI), // The constraints for this path. If using a differential drivetrain, the angular constraints have no effect.
        new IdealStartingState(0, Rotation2d.fromDegrees(0)),
        new GoalEndState(0.0, Rotation2d.fromDegrees(0)) // Goal end state. You can set a holonomic rotation here. If using a differential drivetrain, the rotation will have no effect.
    );

    return newPath;
  }
}
