package frc.robot;

import java.lang.*;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
public class Limelight {
    private static NetworkTable llTable = NetworkTableInstance.getDefault().getTable("limelight");
    //x location of the target
    private static NetworkTableEntry tx = llTable.getEntry("tx");
    //y location of the target
    private static NetworkTableEntry ty = llTable.getEntry("ty");
    //area of the target
    private static NetworkTableEntry ta = llTable.getEntry("ta");
    //does the limelight have a target
    private static NetworkTableEntry tv = llTable.getEntry("tv");

    private double m_targetHeight;
    private double m_limeLightHeight;
    private double m_mountingAngle;


    public Limelight(double targetHeight,double limeLightHeight,double mountingAngle){
        m_targetHeight=targetHeight;
        m_limeLightHeight=limeLightHeight;
        m_mountingAngle=mountingAngle;
    };
    public double getDistance(){
        double distance=0;
        //FROM Limelight Docs
        //d = (h2-h1) / tan(a1+a2)
        double measuredAngle=getY();
        if(measuredAngle>0) {
            distance= (m_targetHeight-m_limeLightHeight) / Math.tan(Math.toRadians(m_mountingAngle + measuredAngle));
        }
        
            return distance;        
    }
    public  double getX(){
        return tx.getDouble(0.0);
    }
    
    public  double getY(){
        return ty.getDouble(0.0);
    }
    public double getArea(){
        return ta.getDouble(0.0);
    }
    
    public static boolean isTargetAvalible(){
        return tv.getBoolean(false);
    }
    public static void ledMode(boolean on){
        double mode = on ? 0 : 1;
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(mode);
    }
    public enum VisionModes{
        LOW(0),
        LEFT(1),
        RIGHT(2);
        public double mode;
        VisionModes(double mode){
            this.mode = mode;
        }
    }
    public static void setVisionMode(){
        setVisionMode(VisionModes.LOW);
    }
    public static void setVisionMode(VisionModes visionMode){
        llTable.getEntry("pipeline").setNumber(visionMode.mode);
    }
    public static String getVisionMode(){
        return  NetworkTableInstance.getDefault().getTable("limelight").getEntry("getpipe").getString("0");
    }
 
    public void update(boolean isEnabled) {
        SmartDashboard.putNumber("limelight x", getX());
        SmartDashboard.putNumber("limelight y", getY());
        SmartDashboard.putNumber("limelight area", getArea());
        SmartDashboard.putNumber("limelight distance", getDistance());
        SmartDashboard.putBoolean("limelight has target", isTargetAvalible());
        SmartDashboard.putString("limelight mode", getVisionMode());
    }
}
