package frc.robot.utilities;

import frc.robot.Constants;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.DoubleArraySubscriber;
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

    private double kConvertInchestoMeters = 0.0254; //Multiple 
    private double m_limeLightHeight;
    private double m_mountingAngle;
    private double m_targetDistance = 0;
    private double m_xStaticOffset = 0;
    private String m_limeLightName;

    double  m_targetHeight=0;

    public Limelight(double targetHeight, double limeLightHeight, double mountingAngle, double xoffSet,
                    double targetDistance,String limeLightName) {
        llTable = NetworkTableInstance.getDefault().getTable(limeLightName);
        tx = llTable.getDoubleTopic("tx").subscribe(0);
        ty = llTable.getDoubleTopic("ty").subscribe(0);
        ta = llTable.getDoubleTopic("ta").subscribe(0);
        tv = llTable.getDoubleTopic("tv").subscribe(0);
        hb = llTable.getDoubleTopic("hb").subscribe(0);
        ledMode = llTable.getDoubleTopic("ledMode");
        pipeline = llTable.getDoubleTopic("pipeline").getEntry(0);
        robotPoseBlue = llTable.getDoubleArrayTopic("botpose_wpiblue").subscribe(new double[] {});
        
        m_targetHeight = targetHeight;
        m_limeLightHeight = limeLightHeight;
        m_mountingAngle = mountingAngle;
        m_xStaticOffset = xoffSet;
        m_targetDistance = targetDistance;
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
        // FROM Limelight Docs
        // d = (h2-h1) / tan(a1+a2)
        double measuredAngle = getY();
        if (measuredAngle != 0) {
           distance = (m_targetHeight - m_limeLightHeight) / Math.tan(Math.toRadians(m_mountingAngle + measuredAngle));
        }   
        return distance;
    }

    public double getDistanceInMeters(){
        return getDistance()*kConvertInchestoMeters;
    }

    public void setPipeline(int pipelineNum){
        pipeline.set(pipelineNum);
    }

    public int getPipeline(){
        return (int) pipeline.get();
    }

    public double getX(){
        return tx.get();
    }

    public double getY(){
        return ty.get();
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

    public void setTargetDistance(double distance) {
        m_targetDistance = distance;
    }

    public void setTargetHeight (double height){
        m_targetHeight=height;
    }

    public boolean DoesLimelightHaveTwoTargets(){
        return tv.get()>= 2.0;
    }
    public double getHeartBeat(){
        return hb.get();
    }
    public Pose2d GetPoseViaApriltag(){
        double[] robotPoseValues=robotPoseBlue.get();

        Pose2d pose =new Pose2d(robotPoseValues[0],robotPoseValues[1],new Rotation2d(0));
        return pose;
    }

    public double[] GetDoublePosArray(){
        double[] empty=null;

        return robotPoseBlue.get();
    }
}