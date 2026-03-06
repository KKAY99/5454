package frc.robot.utilities;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.utilities.MathUtils;
import java.lang.Math;

public class PoseCalculator {

public double getDistance(Pose2d pose1, Pose2d pose2){
// Get the translation components of the poses
Translation2d translation1 = pose1.getTranslation();
Translation2d translation2 = pose2.getTranslation();

// Calculate the distance
double distance = translation1.getDistance(translation2);

System.out.println("Distance: " + distance + " meters");
return distance;
}

public double getBearingAngle(Pose2d pose1, Pose2d pose2){
    double deltaX = pose2.getX() - pose1.getX();
    double deltaY = pose2.getY() - pose1.getY();

    // Angle in radians
    double angleRadians = Math.atan2(deltaY, deltaX);

    // Convert to degrees if needed
    double angleDegrees = Math.toDegrees(angleRadians); 

    System.out.println("Bearing angle (radians): " + angleRadians);
    System.out.println("Bearing angle (degrees): " + angleDegrees);
    
    return angleDegrees;

    // Or use the WPILib utility
//double angleDegreesWPILib = MathUtils.radiansToDegrees(angleRadians); 
    }


}
