package frc.robot.classes;
import java.lang.constant.ConstantDesc;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;
import frc.robot.Constants;
import org.photonvision.targeting.PhotonPipelineResult;
import java.util.List;
import org.photonvision.PhotonUtils;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;

public class PhotonVision {

PhotonCamera m_rightcamera;
PhotonCamera m_leftcamera;

public PhotonVision(){
    m_rightcamera = new PhotonCamera(Constants.rightCamera);
    m_leftcamera = new PhotonCamera(Constants.leftCamera);
}

public boolean CanSeeAnyTarget(){
    var result = m_rightcamera.getLatestResult();
    boolean hasTargets = result.hasTargets();
    return hasTargets;
   
}

public boolean CanSeeTarget(int tagIDtoMatch){
     return getTarget(tagIDtoMatch)!=null;
}

private PhotonTrackedTarget getTarget(int matchTagID){
    boolean foundTarget=false;
    int targetLoop=0;
    PhotonTrackedTarget target=null;
    var result = m_rightcamera.getLatestResult();
    boolean hasTargets = result.hasTargets();
    
    if(hasTargets){
        List<PhotonTrackedTarget> targets = result.getTargets();
        while(foundTarget == false && targetLoop<targets.size()){
            target = targets.get(targetLoop);
            if(target != null){
               if(target.getFiducialId() == matchTagID){
                foundTarget = true;
               }

           }
           targetLoop++;
        }
    }
    return target;   
}


public double GetTargetDistance(int aprilTagID) {
    double range = 0;
    PhotonTrackedTarget target=getTarget(aprilTagID);
    if(target!=null){
        range = PhotonUtils.calculateDistanceToTargetMeters(0.559,Constants.ChargedUp.targetHeightAprilTag,
        0,Units.degreesToRadians(target.getPitch()));
    }
  return range;
}

public double GetX(int matchTagID){
    double X=0;
    PhotonTrackedTarget target = getTarget(matchTagID);
    if(target!=null){
        X = target.getYaw();
    }
    return X;
}

public double GetY(int matchTagID){
    double Y=0;
    PhotonTrackedTarget target = getTarget(matchTagID);
    if(target!=null){
        Y = target.getPitch();
    }
    return Y;
}

public int GetBestMatchAprilTagID () {
    var result = m_rightcamera.getLatestResult();
    List<PhotonTrackedTarget> targets = result.getTargets();
    PhotonTrackedTarget target = result.getBestTarget(); 
    int aprilTagID = target.getFiducialId();

    return aprilTagID;
}

}

