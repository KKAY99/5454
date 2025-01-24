package frc.robot.utilities;

import frc.robot.Constants;

import static edu.wpi.first.units.Units.Newton;

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
    private NetworkTable llTable;
    private DoubleSubscriber tx;
    private DoubleSubscriber ty;
    private DoubleSubscriber ta;
    private DoubleSubscriber tv;
    private DoubleArraySubscriber robotPoseBlue;
    private DoubleArraySubscriber robotPoseBlueOrb;
    private DoubleArraySubscriber rawfiducials;
    private DoubleArraySubscriber targetPoseInRobotSpace;
    private DoubleArrayEntry fiducialIDFilters;
    private DoubleArrayEntry setrobotOrientation;
    private DoubleEntry pipeline;

    private ArrayList<Double> m_fiducialIDFilter=new ArrayList<>();
    private ArrayList<Double> m_previousPoseMean=new ArrayList<>();
    private ArrayList<Pose2d> m_previousRobotPoses=new ArrayList<>();
    private ArrayList<Double> m_previousPoseMeanDiff=new ArrayList<>();

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
        fiducialIDFilters=llTable.getDoubleArrayTopic("fiducial_id_filters_set").getEntry(new double[]{});
        setrobotOrientation=llTable.getDoubleArrayTopic("robot_orientation_set").getEntry(new double []{});
        robotPoseBlue=llTable.getDoubleArrayTopic("botpose_wpiblue").subscribe(new double[]{});
        robotPoseBlueOrb=llTable.getDoubleArrayTopic("botpose_orb_wpiblue").subscribe(new double[]{});
        pipeline=llTable.getDoubleTopic("pipeline").getEntry(0);
        
        m_limeLightHeight=limeLightHeight;
        m_mountingAngle=mountingAngle;

        m_limeLightName=limeLightName;
    };

    public void setTargetHeight(double height){
        m_targetHeight=height;
    }

    public double getDistance() {
        double distance=0;
        double measuredAngle=getY();
        if (measuredAngle!=0) {
            distance=(m_targetHeight-m_limeLightHeight)/Math.tan(Math.toRadians(m_mountingAngle+measuredAngle));
        }   

        return distance;
    }

    public void setCodeIDFilter(int... fiducialIDs){
        m_fiducialIDFilter=new ArrayList<>();

        for(double fiducialID:fiducialIDs){
            m_fiducialIDFilter.add(fiducialID);
        }
    }

    public void resetCodeIDFilter(){
        m_fiducialIDFilter=new ArrayList<>();
    }

    public void setLimelightIDFilter(double... fiducialIDS){
        fiducialIDFilters.set(fiducialIDS);
    }

    public void resetLimelightIDFilter(){
        fiducialIDFilters.set(new double[] {});
    }

    public ArrayList<Double> getRawFiducial(){
        ArrayList<Double> returnArray=new ArrayList<>();
        int numOfFiducials=rawfiducials.get().length/7;

        try{
            for(int i=0;i<numOfFiducials;i++){
                double currentFiducial=99;
                if(rawfiducials.get().length>0+(7*i)&&rawfiducials.get().length!=0){
                    currentFiducial=rawfiducials.get()[0+(7*i)];
                }
                
                if(m_fiducialIDFilter.contains(currentFiducial)){
                    for(int j=i;j<i+7;j++){    
                    returnArray.add(j-i,rawfiducials.get()[j]);
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

    public void setPipeline(int pipelineNum){
        pipeline.set(pipelineNum);
    }

    public int getPipeline(){
        return (int) pipeline.get();
    }

    public double getX(){
        if(!m_fiducialIDFilter.isEmpty()){
            return this.getFiducialX();
        }else{
            return tx.get();
        }
    }

    private double getFiducialX(){
        if(getRawFiducial().get(1)!=null){
            return this.getRawFiducial().get(1);
        }else{
            return 0.0;
        }
        
    }

    public double getY(){
        if(!m_fiducialIDFilter.isEmpty()){
            return this.getFiducialY();
        }else{
            return ty.get();
        }
    }

    private double getFiducialY(){
        if(getRawFiducial().get(2)!=null){
            return this.getRawFiducial().get(2);
        }else{
            return 0.0;
        }
    }

    public boolean isAnyTargetAvailable() {
        return tv.get()==1.0;
    }

    public double GetYawOfAprilTag(){
        if(targetPoseInRobotSpace.get()!=null&&targetPoseInRobotSpace.get().length!=0){
            return this.targetPoseInRobotSpace.get()[4];
        }else{
            return 0.0;
        }
    }

    public boolean isFilteredTargetAvailable(){
        if(!m_fiducialIDFilter.isEmpty()){
            return m_fiducialIDFilter.contains(this.getRawFiducial().get(0))&&tv.get()==1.0;
        }else{
            return false;
        }    
    }

    public void SetRobotOrientation(double yaw,double yawRate){
        setrobotOrientation.set(new double[]{yaw,yawRate,0,0,0,0});
    }

    public Pose2d GetPoseViaMegatag1(){
        double[] robotPoseValues=robotPoseBlue.get();

        Pose2d pose =new Pose2d(robotPoseValues[0],robotPoseValues[1],new Rotation2d(robotPoseValues[2]));
        m_previousRobotPoses.add(pose);

        return pose;
    }

    public Pose2d GetPoseViaMegatag2(){
        double[] robotPoseValues=robotPoseBlueOrb.get();

        Pose2d pose =new Pose2d(robotPoseValues[0],robotPoseValues[1],new Rotation2d(robotPoseValues[2]));
        m_previousRobotPoses.add(pose);

        return pose;
    }

    public void TrimPoseArray(int arraySize){
        ArrayList<Pose2d> newArray=new ArrayList<>();

        if(m_previousRobotPoses.size()>arraySize){
            for(int i=0;i<arraySize;i++){
                newArray.add(this.m_previousRobotPoses.get(m_previousRobotPoses.size()-(i+1)));
            }
    
            m_previousRobotPoses=newArray;
        }
    }

    public boolean GetConfidence(int posesToAverage){
        ArrayList<Double> means=new ArrayList<>();
        ArrayList<Double> newDiff=new ArrayList<>();
        double[] vToAverage;
        double xList=0;
        double yList=0;
        double rotList=0;

        if(m_previousRobotPoses.size()+1>posesToAverage){
            for(int i=m_previousRobotPoses.size()-posesToAverage;i<m_previousRobotPoses.size();i++){
                xList+=m_previousRobotPoses.get(i).getX();
                yList+=m_previousRobotPoses.get(i).getY();
                rotList+=m_previousRobotPoses.get(i).getRotation().getDegrees();
            }

            vToAverage=new double[]{xList,yList,rotList};

            for(int i=0;i<vToAverage.length;i++){
                double mean=vToAverage[i]/posesToAverage;
                means.add(mean);
            }

            if(m_previousPoseMean==null||m_previousPoseMean.size()==0){
                m_previousPoseMean=means;
                //System.out.println("previous pose mean" + m_previousPoseMean);
                return true;
            }

            for(int i=0;i<means.size();i++){
                newDiff.add(m_previousPoseMean.get(i)-means.get(i));
                //System.out.println("new diff" + newDiff);
            }

            if(m_previousPoseMeanDiff.size()==0||m_previousPoseMeanDiff==null){
                m_previousPoseMeanDiff=newDiff;
                //System.out.println("previous pose mean diff"+ m_previousPoseMeanDiff);

                return true;
            }else{
                for(int i=0;i<m_previousPoseMeanDiff.size();i++){
                    double percentDiff=(Math.abs(newDiff.get(i))/Math.abs(m_previousPoseMeanDiff.get(i)))*100;
                    //System.out.println("New Diff " + newDiff);
                    //System.out.println("Previous Pose Mean diff " + m_previousPoseMeanDiff);
                    //System.out.println("percent diff" + percentDiff);

                    if(50.0>percentDiff){
                        //System.out.println("Diffrence to great: "+percentDiff);
                        m_previousPoseMean=means;
                        m_previousPoseMeanDiff=newDiff;
                        return false;
                    }
                }

                m_previousPoseMean=means;
                m_previousPoseMeanDiff=newDiff;
                return true;
            }
        }else{
            return false;
        }

    }
}