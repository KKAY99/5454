package frc.robot.utilities;

import frc.robot.Constants;

import java.util.ArrayList;
import java.util.Collection;

import com.fasterxml.jackson.databind.type.ArrayType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.DoubleArrayTopic;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.IntegerTopic;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.networktables.Topic;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.DoubleEntry;

//@Logged(strategy = Strategy.OPT_IN)
public class Limelight {

    private NetworkTable llTable;
    // x location of the target
    private DoubleSubscriber tx;
    // y location of the target
    private DoubleSubscriber ty;
    // area of the target
    private DoubleSubscriber ta;
    // does the limelight have a target
    private DoubleSubscriber tv;
    //limelight hearbeat
    private DoubleSubscriber hb;
    // robot pose based on limelight
    private DoubleArraySubscriber robotPoseBlue;

    private DoubleTopic ledMode;
    private DoubleEntry pipeline;
    private DoubleArraySubscriber rawfiducials;

    private double m_limeLightHeight;
    private double m_mountingAngle;
    private double  m_targetHeight=0;
    private double m_xStaticOffset = 0;
    private ArrayList<Double> m_fiducialIDFilter=new ArrayList<>();
    private String m_limeLightName;

    public Limelight(double limeLightHeight, double mountingAngle,double xoffSet,String limeLightName) {
        llTable = NetworkTableInstance.getDefault().getTable(limeLightName);
        tx = llTable.getDoubleTopic("tx").subscribe(0);
        ty = llTable.getDoubleTopic("ty").subscribe(0);
        ta = llTable.getDoubleTopic("ta").subscribe(0);
        tv = llTable.getDoubleTopic("tv").subscribe(0);
        hb = llTable.getDoubleTopic("hb").subscribe(0);
        rawfiducials = llTable.getDoubleArrayTopic("rawfiducials").subscribe(new double[] {});
        ledMode = llTable.getDoubleTopic("ledMode");
        pipeline = llTable.getDoubleTopic("pipeline").getEntry(0);
        robotPoseBlue = llTable.getDoubleArrayTopic("botpose_wpiblue").subscribe(new double[] {});
        
        m_limeLightHeight = limeLightHeight;
        m_mountingAngle = mountingAngle;
        m_xStaticOffset = xoffSet;

        m_limeLightName=limeLightName;
    };


    public double getOffset(){
        return m_xStaticOffset;
    }

    public void setOffSet(double newValue){
        m_xStaticOffset=newValue;
    }

    public double getDistance() {
        double distance = 0;
        double measuredAngle = getY();
        if (measuredAngle != 0) {
            distance = (m_targetHeight - m_limeLightHeight) / Math.tan(Math.toRadians(m_mountingAngle + measuredAngle));
        }   

        return distance;
    }

    public void setIDFilter(int... fiducialIDs){
        m_fiducialIDFilter=new ArrayList<>();

        for(double fiducialID:fiducialIDs){
            m_fiducialIDFilter.add(fiducialID);
        }
    }

    public ArrayList<Double> getRawFiducial(){
        ArrayList<Double> returnArray=new ArrayList<>();
        int numOfFiducials=rawfiducials.get().length/7;

        for(int i=0;i<numOfFiducials;i++){
            double currentFiducial=0;
            if(rawfiducials.get().length>0+(7*i)){
                currentFiducial=rawfiducials.get()[0+(7*i)];
            }
            
            if(m_fiducialIDFilter.contains(currentFiducial)){
                for(int j=i;j<i+7;j++){    
                returnArray.add(j-i,rawfiducials.get()[j]);
                }
            }
        }

        if(returnArray.size()==0){
            for(int i=0;i<7;i++){
                returnArray.add(0.0);
            }
        }
        
        return returnArray;
    }

    public void setPipeline(int pipelineNum){
        pipeline.set(pipelineNum);
    }

    public int getPipeline(){
        return (int) pipeline.get();
    }

    public double getX(){
        if(!m_fiducialIDFilter.isEmpty()){
            return this.getRawFiducial().get(1);
        }else{
            return tx.get();
        }
    }

    public double getY(){
        if(!m_fiducialIDFilter.isEmpty()){
            return this.getRawFiducial().get(2);
        }else{
            return ty.get();
        }
    }

    public double getArea() {
        return ta.get();
    }

    public boolean isTargetAvailible() {
        return tv.get() == 1.0;
    }

    public void setLedMode(boolean on) {
        double mode = on ? 0 : 1;
        ledMode.publish().set(mode);
    }

    public enum VisionModes {
        LOW(0),
        LEFT(1),
        RIGHT(2);

        public double mode;

        VisionModes(double mode) {
            this.mode = mode;
        }
    }

    public void setTargetHeight(double height){
        m_targetHeight=height;
    }

    public boolean DoesLimelightHaveTwoTargets(){
        return tv.get()>= 2.0;
    }
    public double getHeartBeat(){
        return hb.get();
    }

    public Pose2d GetPoseViaApriltagNT(){
        double[] robotPoseValues=robotPoseBlue.get();

        Pose2d pose =new Pose2d(robotPoseValues[0],robotPoseValues[1],new Rotation2d(robotPoseValues[2]));
        return pose;
    }

    public Pose2d GetPoseViaHelper(double robotYaw){
        LimelightHelpers.SetRobotOrientation(m_limeLightName,robotYaw,0,0,0,0,0);
        LimelightHelpers.PoseEstimate poseEstimate=LimelightHelpers.getBotPoseEstimate_wpiBlue(m_limeLightName);

        return poseEstimate.pose;
    }   

    public double[] GetDoublePosArray(){
        double[] empty=null;

        return robotPoseBlue.get();
    }
}