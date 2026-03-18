package frc.robot.utilities;

import java.util.ArrayList;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class LimelightManager{
    private Limelight[] m_limeLights;
    private ArrayList<String> m_limeLightNames = new ArrayList<>(); // Fix #3a: was never initialized

    public LimelightManager(Limelight... limeLights){
        m_limeLights=limeLights;

        for(int i=0;i<m_limeLights.length;i++){
            m_limeLightNames.add(m_limeLights[i].getLimelightName());
        }
    }

    private double EuclideanDistance(double x1,double x2,double y1,double y2){
        return Math.sqrt(Math.pow((x1-x2),2)+Math.pow((y1-y2),2));
    }

    public Pose2d MultiCameraPoseEstimation(Pose2d currentPose,int posesToAverage){
        // Fix #3b: use properly sized arrays based on actual limelight count
        int count = m_limeLights.length;
        Pose2d[] limelightsMT2 = new Pose2d[count];
        boolean[] MT2ValShouldUse = new boolean[count];
        double[] confidenceLevelX = new double[count];
        double[] confidenceLevelY = new double[count];
        double xCoordsToAv=0;
        double yCoordsToAv=0;
        double rotToAv=0;
        double diffXToHighConX;
        double diffYToHighConY;
        double newPosX;
        double newPosY;
        double newPosRot;
        int validCount = 0; // Fix #3c: track how many limelights actually reported

        for(int i=0;i<count;i++){
            limelightsMT2[i]=m_limeLights[i].GetPoseViaMegatag2();
            MT2ValShouldUse[i]=m_limeLights[i].getConfidence(posesToAverage,limelightsMT2[i]);

            if(MT2ValShouldUse[i]){
                confidenceLevelX[i]=m_limeLights[i].getLastConfidenceVals()[0];
                confidenceLevelY[i]=m_limeLights[i].getLastConfidenceVals()[1];
                xCoordsToAv+=limelightsMT2[i].getX();
                yCoordsToAv+=limelightsMT2[i].getY();
                rotToAv+=limelightsMT2[i].getRotation().getDegrees();
                validCount++;
            }
        }

        // Fix #3d: divide by validCount (cameras that reported), not total camera count
        newPosX = (validCount > 0) ? xCoordsToAv / validCount : currentPose.getX();
        newPosY = (validCount > 0) ? yCoordsToAv / validCount : currentPose.getY();
        newPosRot = (validCount > 0) ? rotToAv / validCount : currentPose.getRotation().getDegrees();

        int highestConfidencePos=0;
        for(int i=0;i<count;i++){
            highestConfidencePos=(confidenceLevelX[i] > confidenceLevelX[highestConfidencePos]) ? i : highestConfidencePos;
        }

        diffXToHighConX=(Math.abs(newPosX)-Math.abs(limelightsMT2[highestConfidencePos].getX()));
        diffYToHighConY=(Math.abs(newPosY)-Math.abs(limelightsMT2[highestConfidencePos].getY()));

        newPosX=(newPosX-(diffXToHighConX*confidenceLevelX[highestConfidencePos]));
        newPosY=(newPosY-(diffYToHighConY*confidenceLevelY[highestConfidencePos]));
        
        return new Pose2d(newPosX,newPosY,new Rotation2d(newPosRot));
    }
    
    public Pose2d MultiCameraPoseEstimationFullyCustom(){
        return null;
    }
}
