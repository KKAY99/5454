package frc.robot.utilities;

import frc.robot.Constants.LimeLightValues;
import java.util.ArrayList;

import org.littletonrobotics.conduit.schema.SystemData;

import com.ctre.phoenix6.controls.NeutralOut;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.DoubleArrayEntry;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.IntegerEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.DoubleEntry;
import frc.robot.subsystems.CommandSwerveDrivetrain;

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
    private DoubleArraySubscriber botpose_wpiblue;
    //Calculated RobotPose Megatag2
    private DoubleArraySubscriber botpose_orb_wpiblue;
    //Information From All Apriltags
    private DoubleArraySubscriber rawfiducials;
    //Set 3D Offset from Fiducial
    private DoubleArrayEntry fiducial_offset_set;
    //Target Pose Based On Robot
    private DoubleArraySubscriber targetpose_robotspace;
    //Robot Pose Based On Target
    private DoubleArraySubscriber botpose_targetspace;
    //Targets To Look For
    private DoubleArrayEntry fiducial_id_filters_set;
    //Priority Fiducial ID
    private IntegerEntry priorityid;
    //Give Current Gyro Angle
    private DoubleArrayEntry robot_orientation_set;
    //Get/Set Pipeline
    private DoubleEntry pipeline;
    //****

    private ArrayList<Double> m_fiducialIDFilter=new ArrayList<>();
    private ArrayList<Double> m_previousPoseDiffMean=new ArrayList<>();
    private ArrayList<Pose2d> m_previousRobotPoses=new ArrayList<>();
    private ArrayList<ArrayList<Double>> m_previousPoseDiffMeanForAveraging=new ArrayList<>();

    private double[] m_lastConfidenceVals;
    private double m_lastTimeStamp=0;
    private Pose2d m_derivedPose;

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
        priorityid=llTable.getIntegerTopic("priorityid").getEntry(0);
        fiducial_offset_set=llTable.getDoubleArrayTopic("fiducial_offset_set").getEntry(new double[]{});
        targetpose_robotspace=llTable.getDoubleArrayTopic("targetpose_robotspace").subscribe(new double[]{});
        botpose_targetspace=llTable.getDoubleArrayTopic("botpose_targetspace").subscribe(new double[]{});
        fiducial_id_filters_set=llTable.getDoubleArrayTopic("fiducial_id_filters_set").getEntry(new double[]{});
        robot_orientation_set=llTable.getDoubleArrayTopic("robot_orientation_set").getEntry(new double[]{});
        botpose_wpiblue=llTable.getDoubleArrayTopic("botpose_wpiblue").subscribe(new double[]{});
        botpose_orb_wpiblue=llTable.getDoubleArrayTopic("botpose_orb_wpiblue").subscribe(new double[]{});
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
        double measuredAngle=getY(); 

        return (measuredAngle!=0)?(this.m_targetHeight-this.m_limeLightHeight)/Math.tan(Math.toRadians(this.m_mountingAngle+measuredAngle)):
                0;
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
        this.fiducial_id_filters_set.set(fiducialIDS);
    }

    public void resetLimelightIDFilter(){
        this.fiducial_id_filters_set.set(new double[] {});
    }

    public void setPriorityID(int priorityID){
        this.priorityid.set(priorityID);
    }

    public void resetPriorityID(){
        this.priorityid.set(0);
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
        return (!this.m_fiducialIDFilter.isEmpty())?this.getFiducialX():this.tx.get();
    }

    private double getFiducialX(){
        return (this.getRawFiducial().get(1)!=null)?this.getRawFiducial().get(1):0.0;
    }

    public double getY(){
        return (!this.m_fiducialIDFilter.isEmpty())?this.getFiducialY():this.ty.get();
    }

    private double getFiducialY(){
        return (this.getRawFiducial().get(2)!=null)?this.getRawFiducial().get(2):0.0;
    }

    public boolean isAnyTargetAvailable() {
        return this.tv.get()==1.0;
    }

    public double getYawOfAprilTag(){
        return (this.targetpose_robotspace.get()!=null&&this.targetpose_robotspace.get().length!=0)?
                this.targetpose_robotspace.get()[4]:0.0;
    }

    public void resetXYZFiducialOffset(){
        this.fiducial_offset_set.set(new double[]{0,0,0});
    }

    public void setXYZFiducialOffset(double[] coords){
        this.fiducial_offset_set.set(coords);
    }

    public double[] getBotPoseTargetSpace(){
        return (this.botpose_targetspace.get()!=null&&this.botpose_targetspace.get().length!=0)?
                this.botpose_targetspace.get():new double[]{};
    }

    public ArrayList<double[]> getBotPoseInAllTargetSpaces(){
        double[] previousFilters=this.fiducial_id_filters_set.get();
        ArrayList<double[]> botPoses=new ArrayList<double[]>();
        this.resetCodeIDFilter();

        double[] currentFiducials=this.getAllVisibleFiducialIDs();
        for(int i=0;i<currentFiducials.length;i++){
            this.setCodeIDFilter(currentFiducials[i]);
            botPoses.add(this.getBotPoseTargetSpace());
        }

        if(previousFilters!=null){
            this.setCodeIDFilter(previousFilters);
        }
        
        return botPoses;
    }

    public Pose2d findGlobalPoseFromTargetPoseRobotSpace(double gyroAngle,LimeLightValues.LimelightLineUpOffsets offsetState){
        double[] targetPoseRobotSpace=this.targetpose_robotspace.get();
        Pose2d botPose=this.GetPoseViaMegatag2();
        double offsetX=0;
        double offsetY=0;

        if(targetPoseRobotSpace[0]!=0){
            switch(offsetState){
                case CENTER:
                offsetX=LimeLightValues.odomLineUpXOffsetCenter;
                offsetY=LimeLightValues.odomLineUpYOffsetCenter;
                break;
                case LEFT:
                offsetX=LimeLightValues.odomLineUpXOffsetLeft;
                offsetY=LimeLightValues.odomLineUpYOffsetLeft;
                break;
                case RIGHT:
                offsetX=LimeLightValues.odomLineUpXOffsetRight;
                offsetY=LimeLightValues.odomLineUpYOffsetRight;
                break;
            }
    
            double newX=botPose.getX()+(targetPoseRobotSpace[2]+offsetX);
            double newY=botPose.getY()-(targetPoseRobotSpace[0]+offsetY);
            double newRot=gyroAngle+this.getYawOfAprilTag();
    
            return new Pose2d(newX,newY,new Rotation2d().fromDegrees(newRot));

        }else{
            return null;
        }
    }

    public boolean isFilteredTargetAvailable(){
        return (!this.m_fiducialIDFilter.isEmpty())?this.m_fiducialIDFilter.contains(this.getRawFiducial().get(0)
        )&&this.tv.get()==1.0:false;
    }

    public void SetRobotOrientation(double yaw,double yawRate){
        this.robot_orientation_set.set(new double[]{yaw,yawRate,0,0,0,0});
    }

    public Pose2d GetPoseViaMegatag1(){
        double[] robotPoseValues=this.botpose_wpiblue.get();
        Pose2d pose;

        if(robotPoseValues!=null){
        pose =new Pose2d(robotPoseValues[0],robotPoseValues[1],new Rotation2d().fromDegrees(robotPoseValues[5]));
        m_previousRobotPoses.add(pose);
        }else{
            pose=new Pose2d(0,0,new Rotation2d(0));
        }

        return pose;
    }

    public Pose2d GetPoseViaMegatag2(){
        double[] robotPoseValues=this.botpose_orb_wpiblue.get();
        Pose2d pose;

        if(robotPoseValues!=null&&robotPoseValues.length!=0){
            pose=new Pose2d(robotPoseValues[0],robotPoseValues[1],new Rotation2d().fromDegrees(robotPoseValues[5]));
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

    public double[] getLastConfidenceVals(){
        boolean check=(m_lastConfidenceVals.length!=0);
        
        return check?m_lastConfidenceVals:new double[] {};
    }

    public boolean getConfidence(int posesToAverage,Pose2d currentVisionPose){
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
                    this.m_lastConfidenceVals= new double[]{((((Math.abs(100-percentDiffX)))/
                        (LimeLightValues.confidenceDeadbandMax-LimeLightValues.confidenceDeadbandMin))*100),
                        ((((Math.abs(100-percentDiffY)))/
                        (LimeLightValues.confidenceDeadbandMax-LimeLightValues.confidenceDeadbandMin))*100)};

                    returnValue=true;
                }else{
                    returnValue=false;
                }

                this.m_previousPoseDiffMean=diffMeans;
            }
        }

        return returnValue;
    }

    public boolean getDerivationConfidence(CommandSwerveDrivetrain swerve,int posesToAverage,Pose2d currentVisionPose,double currentTimeStamp){
        boolean returnValue=false;
        double timeStampDiff=(m_lastTimeStamp!=0)?m_lastTimeStamp-currentTimeStamp:0;
        double currentXMPS=swerve.getChassisSpeeds().vxMetersPerSecond;
        double currentYMPS=swerve.getChassisSpeeds().vyMetersPerSecond;
        double currentRotROC=swerve.getPigeon2().getYaw().getValueAsDouble();
        double xDisplacement=currentVisionPose.getX()+((currentXMPS*timeStampDiff)/LimeLightValues.cartPointToMeterMult);
        double yDisplacement=currentVisionPose.getY()+((currentYMPS*timeStampDiff)/LimeLightValues.cartPointToMeterMult);
        double rotDisplacement=currentVisionPose.getRotation().getDegrees()+(currentRotROC*timeStampDiff);
        
        m_lastTimeStamp=currentTimeStamp;

        if(m_derivedPose==null){
            m_derivedPose=new Pose2d(xDisplacement,yDisplacement,new Rotation2d().fromDegrees(rotDisplacement));
        }else{
            double xMetersDiff=(Math.abs(currentVisionPose.getX()-Math.abs(m_derivedPose.getX())))*LimeLightValues.cartPointToMeterMult;
            double yMetersDiff=(Math.abs(currentVisionPose.getY()-Math.abs(m_derivedPose.getY())))*LimeLightValues.cartPointToMeterMult;
            double rotDiff=Math.abs(m_derivedPose.getRotation().getDegrees())-Math.abs(currentVisionPose.getRotation().getDegrees());

            m_derivedPose=new Pose2d(xDisplacement,yDisplacement,new Rotation2d().fromDegrees(rotDisplacement));
            returnValue=(xMetersDiff<LimeLightValues.maxMeterDiff)&&(yMetersDiff<LimeLightValues.maxMeterDiff)&&(rotDiff<LimeLightValues.maxRotDiff);
        }

        return returnValue;
    }
}