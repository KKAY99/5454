package frc.robot.utilities;

import java.util.ArrayList;
import edu.wpi.first.math.geometry.Pose2d;

public class LimelightManager{
    private Limelight[] m_limeLights;
    private ArrayList<String> m_limeLightNames;

    public LimelightManager(Limelight... limeLights){
        m_limeLights=limeLights;

        for(int i=0;i<m_limeLights.length;i++){
            m_limeLightNames.add(m_limeLights[i].getLimelightName());
        }
    }

    private double EuclideanDistance(double x1,double x2,double y1,double y2){
        return Math.sqrt(Math.pow((x1-x2),2)+Math.pow((y1-y2),2));
    }

    public Pose2d MultiCameraPoseEstimation(){

        return null;
    }
    
}
