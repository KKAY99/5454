package frc.robot.utilities;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants.AutoConstants;
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
        Pose2d speakerPos;

        if(DriverStation.getAlliance().get()==Alliance.Red){
            speakerPos=AutoConstants.redSpeakerPos;
        }else{
            speakerPos=AutoConstants.blueSpeakerPos;
        }

        return speakerPos;
    }
}
