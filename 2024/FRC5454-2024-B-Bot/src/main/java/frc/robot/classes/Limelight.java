package frc.robot.classes;

import frc.robot.Constants;

import java.util.Map;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Limelight {

    private static NetworkTable llTable;
    // x location of the target
    private static NetworkTableEntry tx;
    // y location of the target
    private static NetworkTableEntry ty;
    // area of the target
    private static NetworkTableEntry ta;
    // does the limelight have a target
    private static NetworkTableEntry tv;


    private double m_limeLightHeight;
    private double m_mountingAngle;
    private double m_targetDistance = 0;
    private double m_xStaticOffset = 0;
    private boolean m_LimelightLEDOn = false;

    private double kP = Constants.LimeLightValues.steeringP;
    private double kI = Constants.LimeLightValues.steeringI;
    private double kD = Constants.LimeLightValues.steeringD;
    private double kFeedForward = Constants.LimeLightValues.steeringFeedForward;

    PIDController limeLightSteeringController = new PIDController(kP, kI, kD);

    boolean m_dynamicEnabled = false;
    double  m_targetHeight=0;
    public Limelight() {
        this(0.0, 0.0, 0.0);
    };

    public Limelight(double targetHeight, double limeLightHeight, double mountingAngle) {
        this(targetHeight, limeLightHeight, mountingAngle, Constants.LimeLightValues.kVisionXOffset);
    };

    public Limelight(double targetHeight, double limeLightHeight, double mountingAngle, double xoffSet) {
        this(targetHeight, limeLightHeight, mountingAngle, xoffSet,0 );
    };
    public Limelight(double targetHeight, double limeLightHeight, double mountingAngle, double xoffSet,
    double targetDistance) {
        //Default Name to limelight 
        this(targetHeight, limeLightHeight, mountingAngle, xoffSet,targetDistance,"limelight" );
        
    }

    public Limelight(double targetHeight, double limeLightHeight, double mountingAngle, double xoffSet,
            double targetDistance,String limeLightName) {
        
        llTable = NetworkTableInstance.getDefault().getTable(limeLightName);
        // x location of the target
        tx = llTable.getEntry("tx");
        // y location of the target
        ty = llTable.getEntry("ty");
        // area of the target
        ta = llTable.getEntry("ta");
        // does the limelight have a target
        tv = llTable.getEntry("tv");
        
        m_targetHeight = targetHeight;
        m_limeLightHeight = limeLightHeight;
        m_mountingAngle = mountingAngle;
        m_xStaticOffset = xoffSet;
        m_targetDistance = targetDistance;
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
        //System.out.println(distance + " - " + m_targetHeight + " -- " + m_limeLightHeight + " --- " + measuredAngle + "----" + m_mountingAngle);
        return distance;
    }
    public void setPipeline(int pipeline){
        llTable.getEntry("pipeline").setNumber(pipeline);
        
        
    }
    public int getPipeline(){
        return (int) llTable.getEntry("pipeline").getInteger(99);
    }

    public double getRotationPower(double measurement) {
        double returnVal = getRotationPower(measurement, 0.0);
        // System.out.println("LL Steer: " + returnVal);
        return returnVal;
    }

    public double getRotationPower(double measurement, double setpoint) {
        if (isTargetAvailible()) {
            return limeLightSteeringController.calculate(measurement, setpoint) + kFeedForward;
        }
        return 0.0;
    }


     public double getXRaw(){
        return tx.getDouble(0.0);
     }
    public double getX() {
 //       if (m_dynamicEnabled) {
//            return tx.getDouble(0.0) + getOffset(offsetValues, getDistance());
 //       } else {
            return tx.getDouble(0.0) + m_xStaticOffset;
 //       }
    }

    public double getactualX() {
        return tx.getDouble(0.0);
    }

    public double getY() {
        return ty.getDouble(0.0);
    }
    public double getYRaw(){
        return ty.getDouble(0.0);
    }
    

    public double getArea() {
        return ta.getDouble(0.0);
    }

    public boolean isTargetAvailible() {

        return tv.getDouble(0) == 1.0;
    }

    public static void ledMode(boolean on) {
        double mode = on ? 0 : 1;
        llTable.getEntry("ledMode").setNumber(mode);
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

    public static void setVisionMode() {
        setVisionMode(VisionModes.LOW);
    }

    public static void setVisionMode(VisionModes visionMode) {
        llTable.getEntry("pipeline").setNumber(visionMode.mode);
    }

    public static String getVisionMode() {
        return llTable.getEntry("getpipe").getString("0");
    }

    public void setTargetDistance(double distance) {
        m_targetDistance = distance;
    }
    public void setTargetHeight (double height){
        m_targetHeight=height;
    }

    private boolean isAtTargetDistance() {
        boolean returnValue = false;
        if (m_targetDistance > 0) {
            if ((Math.abs(m_targetDistance - getDistance()) < Constants.LimeLightValues.kVisionDistanceTolerance)) {
                returnValue = true;
            }
        }
        return returnValue;
    }

    public boolean isOnTargetX() {
        boolean returnValue = false;
        if (isTargetAvailible()) {
            if ((Math.abs(getX()) < Constants.LimeLightValues.kVisionXTolerance)) {
                // System.out.println("On Target -" + Math.abs(getX()
                // ) + " - " + Constants.LimeLightValues.kVisionXTolerance);
                returnValue = true;
            } else {
                // System.out.println("Off Target -" + Math.abs(getX()
                // ) + " - " + Constants.LimeLightValues.kVisionXTolerance);

            }

        } else {
            // System.out.println("Off Target -" + Math.abs(getX()
            // ) + " - " + Constants.LimeLightValues.kVisionXTolerance);

        }

        return returnValue;

    }

    private void updateDashboard() {
        SmartDashboard.putNumber("limelight x", getX());
        SmartDashboard.putNumber("limelight y", getY());
        SmartDashboard.putNumber("limelight area", getArea());
        SmartDashboard.putNumber("Our limelight distance", getDistance());
        SmartDashboard.putBoolean("limelight has target", isTargetAvailible());
        SmartDashboard.putBoolean("Target Distance", isAtTargetDistance());
        SmartDashboard.putBoolean("On TargetX", isOnTargetX());
        SmartDashboard.putString("limelight mode", getVisionMode());

    }

    public void turnLEDOff() {
        llTable.getEntry("ledMode").setNumber(1);
        m_LimelightLEDOn = false;
    }

    public void turnLEDOn() {
        // turn off lED
        llTable.getEntry("ledMode").setNumber(3);
        m_LimelightLEDOn = true;
    }

    public void update(boolean isEnabled) {

        updateDashboard();

    }


    public void update() {
        if (m_LimelightLEDOn) {
            updateDashboard();
        }
    }
}
