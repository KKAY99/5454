package frc.robot.classes;

import frc.robot.Constants;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Limelight {
    private static NetworkTable llTable = NetworkTableInstance.getDefault().getTable("limelight");
    // x location of the target
    private static NetworkTableEntry tx = llTable.getEntry("tx");
    // y location of the target
    private static NetworkTableEntry ty = llTable.getEntry("ty");
    // area of the target
    private static NetworkTableEntry ta = llTable.getEntry("ta");
    // does the limelight have a target
    private static NetworkTableEntry tv = llTable.getEntry("tv");

    private double m_targetHeight;
    private double m_limeLightHeight;
    private double m_mountingAngle;
    private double m_targetDistance = 0;
    private double m_xOffset=0;
    private boolean m_LimelightLEDOn = false;

    private double kP = Constants.LimeLightValues.steeringP;
    private double kI = Constants.LimeLightValues.steeringI;
    private double kD = Constants.LimeLightValues.steeringD;
    private double kFeedForward = Constants.LimeLightValues.steeringFeedForward;

    PIDController limeLightSteeringController = new PIDController(kP, kI, kD);

    public Limelight() {
        m_targetHeight = 0;
        m_limeLightHeight = 0;
        m_mountingAngle = 0;
    };

    public Limelight(double targetHeight, double limeLightHeight, double mountingAngle) {
        m_targetHeight = targetHeight;
        m_limeLightHeight = limeLightHeight;
        m_mountingAngle = mountingAngle;
    };
    
    public Limelight(double targetHeight, double limeLightHeight, double mountingAngle, double xoffSet) {
        m_targetHeight = targetHeight;
        m_limeLightHeight = limeLightHeight;
        m_mountingAngle = mountingAngle;
        m_xOffset=xoffSet;
    };

    public Limelight(double targetHeight, double limeLightHeight, double mountingAngle,double xoffSet, double targetDistance) {
        m_targetHeight = targetHeight;
        m_limeLightHeight = limeLightHeight;
        m_mountingAngle = mountingAngle;
        m_xOffset=xoffSet;
        m_targetDistance = targetDistance;
    };

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

    // public double getStafePower(double error) {
    // //Adjust the coefficient to adjust power don't excede 0.14 maybe?
    // return 0.145*Math.pow(Math.abs(error), 0.5);
    // }

    // public double getFwdPower(double error) {
    // //Adjust the coefficient to adjust power don't excede 0.06 maybe?
    // return 0.115*Math.pow(Math.abs(error), 0.5);
    // }

    public double getX() {
        
        return tx.getDouble(0.0)-m_xOffset;
    }
    public double getactualX() {
        return tx.getDouble(0.0);
    }

    public double getY() {
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
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(mode);
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
        return NetworkTableInstance.getDefault().getTable("limelight").getEntry("getpipe").getString("0");
    }

    public void setTargetDistance(double distance) {
        m_targetDistance = distance;
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

    private boolean isOnTargetX() {
        boolean returnValue = false;
        if (isTargetAvailible()) {
            if ((Math.abs(getX()
                    - Constants.LimeLightValues.targetXPosShoot) < Constants.LimeLightValues.kVisionXTolerance)) {
                System.out.println("On Target -" + Math.abs(getX()- Constants.LimeLightValues.targetXPosShoot
                    ) + " - " + Constants.LimeLightValues.kVisionXTolerance);
                        returnValue = true;
            }
        }
        return returnValue;

    }

    private void updateDashboard() {
        SmartDashboard.putNumber("limelight x", getX());
        SmartDashboard.putNumber("limelight y", getY());
        SmartDashboard.putNumber("limelight area", getArea());
        SmartDashboard.putNumber("limelight distance", getDistance());
        SmartDashboard.putBoolean("limelight has target", isTargetAvailible());
        SmartDashboard.putBoolean("Target Distance", isAtTargetDistance());
        SmartDashboard.putBoolean("On TargetX", isOnTargetX());
        SmartDashboard.putString("limelight mode", getVisionMode());

    }

    public void turnLEDOff() {
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(1);
        m_LimelightLEDOn = false;
    }

    public void turnLEDOn() {
        // turn off lED
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(3);
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
