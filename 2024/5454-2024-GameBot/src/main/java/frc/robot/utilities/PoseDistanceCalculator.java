package frc.robot.utilities;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

import java.lang.Math;

public class PoseDistanceCalculator{

    public PoseDistanceCalculator(){}

    public double getDistance(Pose2d currentPos){
        Pose2d alliancePos=GetAlliancePose2d();

        double changeInX=ChangeInX(currentPos.getX(),alliancePos.getX());
        double changeInY=ChangeInY(currentPos.getY(),alliancePos.getY());

        double distance=Math.sqrt(((changeInX*changeInX)+(changeInY*changeInY)));

        return distance;
    }
    
    public double ConvertDistance(){
        return 0.0;
    }

    private double ChangeInX(double currentPosX,double targetPosX){
        double rightTriBase;
        rightTriBase=targetPosX-currentPosX;

        return Math.abs(rightTriBase);
    }

    private double ChangeInY(double currentPosY,double targetPosY){
        double rightTriLeg;
        rightTriLeg=targetPosY-currentPosY;
        
        return Math.abs(rightTriLeg);
    }

    private Pose2d GetAlliancePose2d(){
        return new Pose2d(0,0,new Rotation2d(0));
    }
}
