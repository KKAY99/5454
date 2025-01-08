package frc.robot.utilities;

import java.util.ArrayList;
import java.util.List;
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

public class AutoPlanner {
    
    public AutoPlanner(){

    }

    public PathPlannerPath CreateAutoPath(double endPosRot,Pose2d... poses){
    List<PathPoint> poseArray=new ArrayList<PathPoint>();

    for(Pose2d pose:poses){
      if(pose!=null){
        poseArray.add(new PathPoint(new Translation2d(pose.getX(),pose.getY())));
      }
    }

    PathPlannerPath newPath = PathPlannerPath.fromPathPoints(
        poseArray,
        new PathConstraints(2.0, 2.0, 11.5, 11.5), // The constraints for this path. If using a differential drivetrain, the angular constraints have no effect.
        new GoalEndState(0.0, Rotation2d.fromDegrees(endPosRot)) // Goal end state. You can set a holonomic rotation here. If using a differential drivetrain, the rotation will have no effect.
    );

    if(DriverStation.getAlliance().get()==Alliance.Blue){
      return newPath;
    }else{
      return newPath.flipPath();
    }
  }
}
