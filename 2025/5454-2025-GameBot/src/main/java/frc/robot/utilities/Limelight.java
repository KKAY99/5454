package frc.robot.utilities;

import frc.robot.Constants;
import frc.robot.Constants.LimeLightValues;
import java.net.http.HttpResponse.BodySubscriber;
import java.util.ArrayList;
import java.util.Collection;
import com.fasterxml.jackson.databind.type.ArrayType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.DoubleArrayEntry;
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
    //****Limelight Network Table Values****
    //Limelight Network Table
    private NetworkTable llTable;
    //X Value
    private DoubleSubscriber tx;
    //Y Value
    private DoubleSubscriber ty;
    //Area Value
    private DoubleSubscriber ta;
    //Can See a Target
    private DoubleSubscriber tv;
    //Calculated RobotPose Megatag1
    private DoubleArraySubscriber robotPoseBlue;
    //Calculated RobotPose Megatag2
    private DoubleArraySubscriber robotPoseBlueOrb;
    //Information From All Apriltags
    private DoubleArraySubscriber rawfiducials;
    //Target Pose Based On Robot
    private DoubleArraySubscriber targetPoseInRobotSpace;
    //Robot Pose Based On Target
    private DoubleArraySubscriber botPoseInTargetSpace;
    //Targets To Look For
    private DoubleArrayEntry fiducialIDFilters;
    //Give Current Gyro Angle
    private DoubleArrayEntry setrobotOrientation;
    //Get/Set Pipeline
    private DoubleEntry pipeline;
    //****

    private ArrayList<Double> m_fiducialIDFilter=new ArrayList<>();
    private ArrayList<Double> m_previousPoseDiffMean=new ArrayList<>();
    private ArrayList<Pose2d> m_previousRobotPoses=new ArrayList<>();
    private ArrayList<ArrayList<Double>> m_previousPoseDiffMeanForAveraging=new ArrayList<>();

    private double m_limeLightHeight;
    private double m_mountingAngle;
    private double m_targetHeight=0;
    
    private String m_limeLightName;

    public Limelight(double limeLightHeight,double mountingAngle,double xoffSet,String limeLightName) {
        llTable = NetworkTableInstance.getDefault().getTable(limeLightName);
        tx=llTable.getDoubleTopic("tx").subscribe(0);
        ty=llTable.getDoubleTopic("ty").subscribe(0);
        ta=llTable.getDoubleTopic("ta").subscribe(0);
        tv=llTable.getDoubleTopic("tv").subscribe(0);
        rawfiducials=llTable.getDoubleArrayTopic("rawfiducials").subscribe(new double[] {});
        targetPoseInRobotSpace=llTable.getDoubleArrayTopic("targetpose_robotspace").subscribe(new double[]{});
        botPoseInTargetSpace=llTable.getDoubleArrayTopic("botpose_targetspace").subscribe(new double[]{});
        fiducialIDFilters=llTable.getDoubleArrayTopic("fiducial_id_filters_set").getEntry(new double[]{});
        setrobotOrientation=llTable.getDoubleArrayTopic("robot_orientation_set").getEntry(new double[]{});
        robotPoseBlue=llTable.getDoubleArrayTopic("botpose_wpiblue").subscribe(new double[]{});
        robotPoseBlueOrb=llTable.getDoubleArrayTopic("botpose_orb_wpiblue").subscribe(new double[]{});
        pipeline=llTable.getDoubleTopic("pipeline").getEntry(0);
        
        m_limeLightHeight=limeLightHeight;
        m_mountingAngle=mountingAngle;

        m_limeLightName=limeLightName;
    };

    public String getLimelightName(){
        return this.m_limeLightName;
    }

    public void setTargetHeight(double height){
        this.m_targetHeight=height;
    }

    public double getDistance() {
        double distance=0;
        double measuredAngle=getY();
        if (measuredAngle!=0) {
            distance=(this.m_targetHeight-this.m_limeLightHeight)/Math.tan(Math.toRadians(this.m_mountingAngle+measuredAngle));
        }   

        return distance;
    }

    public void setCodeIDFilter(double... fiducialIDs){
        this.m_fiducialIDFilter=new ArrayList<>();

        for(double fiducialID:fiducialIDs){
            this.m_fiducialIDFilter.add(fiducialID);
        }
    }

    public void resetCodeIDFilter(){
        this.m_fiducialIDFilter=new ArrayList<>();
    }

    public void setLimelightIDFilter(double... fiducialIDS){
        this.fiducialIDFilters.set(fiducialIDS);
    }

    public void resetLimelightIDFilter(){
        this.fiducialIDFilters.set(new double[] {});
    }

    public ArrayList<Double> getRawFiducial(){
        ArrayList<Double> returnArray=new ArrayList<>();
        int numOfFiducials=rawfiducials.get().length/7;

        try{
            for(int i=0;i<numOfFiducials;i++){
                double currentFiducial=99;
                if(this.rawfiducials.get().length>0+(7*i)&&this.rawfiducials.get().length!=0){
                    currentFiducial=this.rawfiducials.get()[0+(7*i)];
                }
                
                if(this.m_fiducialIDFilter.contains(currentFiducial)){
                    for(int j=i;j<i+7;j++){    
                    returnArray.add(j-i,this.rawfiducials.get()[j]);
                    }
                }
            }
        }catch(Exception e){}

        if(returnArray.size()==0){
            for(int i=0;i<8;i++){
                returnArray.add(0.0);
            }
        }
        
        return returnArray;
    }

    public double[] getAllVisibleFiducialIDs(){
        int numOfFiducials=rawfiducials.get().length/7;
        double[] fiducialIDs=new double[]{};

        for(int i=0;i<numOfFiducials;i++){
            fiducialIDs[i]=this.rawfiducials.get()[0+(7*i)];
        }

        return fiducialIDs;
    }

    public void setPipeline(int pipelineNum){
        this.pipeline.set(pipelineNum);
    }

    public int getPipeline(){
        return (int) this.pipeline.get();
    }

    public double getX(){
        if(!this.m_fiducialIDFilter.isEmpty()){
            return this.getFiducialX();
        }else{
            return this.tx.get();
        }
    }

    private double getFiducialX(){
        if(this.getRawFiducial().get(1)!=null){
            return this.getRawFiducial().get(1);
        }else{
            return 0.0;
        }
        
    }

    public double getY(){
        if(!this.m_fiducialIDFilter.isEmpty()){
            return this.getFiducialY();
        }else{
            return ty.get();
        }
    }

    private double getFiducialY(){
        if(this.getRawFiducial().get(2)!=null){
            return this.getRawFiducial().get(2);
        }else{
            return 0.0;
        }
    }

    public boolean isAnyTargetAvailable() {
        return this.tv.get()==1.0;
    }

    public double getYawOfAprilTag(){
        if(this.targetPoseInRobotSpace.get()!=null&&this.targetPoseInRobotSpace.get().length!=0){
            return this.targetPoseInRobotSpace.get()[4];
        }else{
            return 0.0;
        }
    }

    public double[] getBotPoseInTargetSpace(){
        if(this.botPoseInTargetSpace.get()!=null&&this.botPoseInTargetSpace.get().length!=0){
            return this.botPoseInTargetSpace.get();
        }else{
            return new double[]{};
        }
    }

    public ArrayList<double[]> getBotPoseInAllTargetSpaces(){
        double[] previousFilters=this.fiducialIDFilters.get();
        ArrayList<double[]> botPoses=new ArrayList<double[]>();
        this.resetCodeIDFilter();

        double[] currentFiducials=this.getAllVisibleFiducialIDs();
        for(int i=0;i<currentFiducials.length;i++){
            this.setCodeIDFilter(currentFiducials[i]);
            botPoses.add(this.getBotPoseInTargetSpace());
        }

        if(previousFilters!=null||previousFilters.length!=0){
            this.setCodeIDFilter(previousFilters);
        }
        
        return botPoses;
    }

    public boolean isFilteredTargetAvailable(){
        if(!this.m_fiducialIDFilter.isEmpty()){
            return this.m_fiducialIDFilter.contains(this.getRawFiducial().get(0))&&this.tv.get()==1.0;
        }else{
            return false;
        }    
    }

    public void SetRobotOrientation(double yaw,double yawRate){
        this.setrobotOrientation.set(new double[]{yaw,yawRate,0,0,0,0});
    }

    public Pose2d GetPoseViaMegatag1(){
        double[] robotPoseValues=this.robotPoseBlue.get();

        Pose2d pose =new Pose2d(robotPoseValues[0],robotPoseValues[1],new Rotation2d(robotPoseValues[2]));
        m_previousRobotPoses.add(pose);

        return pose;
    }

    public Pose2d GetPoseViaMegatag2(){
        double[] robotPoseValues=this.robotPoseBlueOrb.get();
        Pose2d pose;

        if(robotPoseValues!=null||robotPoseValues.length!=0){
            pose =new Pose2d(robotPoseValues[0],robotPoseValues[1],new Rotation2d(robotPoseValues[2]));
            m_previousRobotPoses.add(pose);
        }else{
            pose=new Pose2d(0,0,new Rotation2d(0));
        }

        return pose;
    }

    public void TrimPoseArray(int arraySize){
        ArrayList<Pose2d> newArray=new ArrayList<>();

        if(this.m_previousRobotPoses.size()>arraySize*2){
            for(int i=0;i<arraySize;i++){
                newArray.add(this.m_previousRobotPoses.get(this.m_previousRobotPoses.size()-(i+1)));
            }
    
            this.m_previousRobotPoses=newArray;
        }
    }

    public double[] AverageOfAllDiffMeans(){
        double xAverage=0;
        double yAverage=0;

        for(int i=0;i<this.m_previousPoseDiffMeanForAveraging.size();i++){
            xAverage+=this.m_previousPoseDiffMeanForAveraging.get(i).get(0);
            yAverage+=this.m_previousPoseDiffMeanForAveraging.get(i).get(1);
        }

        xAverage=xAverage/this.m_previousPoseDiffMeanForAveraging.size();
        yAverage=xAverage/this.m_previousPoseDiffMeanForAveraging.size();

        return new double[]{xAverage,yAverage};
    }

    public boolean GetConfidence(int posesToAverage,Pose2d currentVisionPose){
        boolean returnValue=false;
        ArrayList<Double> diffMeans=new ArrayList<>();
        double[] vToAverage;
        double xDiffList=0;
        double yDiffList=0;

        if(this.m_previousRobotPoses.size()+1>posesToAverage){
            for(int i=this.m_previousRobotPoses.size()-posesToAverage;i<this.m_previousRobotPoses.size();i++){
                xDiffList+=Math.abs((Math.abs(this.m_previousRobotPoses.get(i).getX())-Math.abs(currentVisionPose.getX())));
                yDiffList+=Math.abs((Math.abs(this.m_previousRobotPoses.get(i).getY())-Math.abs(currentVisionPose.getY())));
            }

            vToAverage=new double[]{xDiffList,yDiffList};

            for(int i=0;i<vToAverage.length;i++){
                double mean=vToAverage[i]/posesToAverage;
                diffMeans.add(mean);
            }

            if(this.m_previousPoseDiffMean==null||this.m_previousPoseDiffMean.size()==0){
                this.m_previousPoseDiffMean=diffMeans;
                returnValue=true;
            }else{
                double percentDiffX=((Math.abs(diffMeans.get(0)))/Math.abs(this.m_previousPoseDiffMean.get(0)))*100;
                double percentDiffY=((Math.abs(diffMeans.get(1)))/Math.abs(this.m_previousPoseDiffMean.get(0)))*100;

                if(LimeLightValues.confidenceDeadbandMin<percentDiffX&&LimeLightValues.confidenceDeadbandMin<percentDiffY&&
                    LimeLightValues.confidenceDeadbandMax>percentDiffX&&LimeLightValues.confidenceDeadbandMax>percentDiffY){
                    returnValue=true;
                }else{
                    returnValue=false;
                }

                this.m_previousPoseDiffMean=diffMeans;
            }
        }

            return returnValue;
    }
}