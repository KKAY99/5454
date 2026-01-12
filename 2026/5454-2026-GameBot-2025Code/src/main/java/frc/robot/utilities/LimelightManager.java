package frc.robot.utilities;

import java.util.ArrayList;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

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

    public Pose2d MultiCameraPoseEstimation(Pose2d currentPose,int posesToAverage){
        Pose2d[] limelightsMT2=new Pose2d[]{};
        boolean[] MT2ValShouldUse=new boolean[]{};
        double[] confidenceLevelX=new double[]{};
        double[] confidenceLevelY=new double[]{};
        double xCoordsToAv=0;
        double yCoordsToAv=0;
        double rotToAv=0;
        double diffXToHighConX;
        double diffYToHighConY;
        double newPosX;
        double newPosY;
        double newPosRot;

        for(int i=0;i<m_limeLights.length;i++){
            limelightsMT2[i]=m_limeLights[i].GetPoseViaMegatag2();
            MT2ValShouldUse[i]=m_limeLights[i].getConfidence(posesToAverage,limelightsMT2[i]);

            if(MT2ValShouldUse[i]){
                confidenceLevelX[i]=m_limeLights[i].getLastConfidenceVals()[0];
                confidenceLevelY[i]=m_limeLights[i].getLastConfidenceVals()[1];
                xCoordsToAv+=m_limeLights[i].GetPoseViaMegatag2().getX();
                yCoordsToAv+=m_limeLights[i].GetPoseViaMegatag2().getY();
                rotToAv+=m_limeLights[i].GetPoseViaMegatag2().getRotation().getDegrees();
            }
        }

        newPosX=(xCoordsToAv!=0)?xCoordsToAv/m_limeLights.length:currentPose.getX();
        newPosY=(yCoordsToAv!=0)?yCoordsToAv/m_limeLights.length:currentPose.getY();
        newPosRot=(rotToAv!=0)?rotToAv/m_limeLights.length:currentPose.getRotation().getDegrees();

        int highestConfidencePos=0;
        for(int i=0;i<m_limeLights.length;i++){
            highestConfidencePos=(highestConfidencePos==0)?i:
            (highestConfidencePos<=confidenceLevelX[i])?i:highestConfidencePos;
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
