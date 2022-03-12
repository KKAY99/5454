package frc.robot.classes;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;

public class Vision {

    static NetworkTableInstance visionTable = NetworkTableInstance.getDefault();
    static NetworkTable myCam = visionTable.getTable("chameleon-vision").getSubTable("PsCam");
    static NetworkTable openSight = visionTable.getTable("OpenSight");

    private static NetworkTableEntry NTballPositions = openSight.getEntry("Circles-x");
    private static NetworkTableEntry NTballSizes = openSight.getEntry("Circles-radius");

    private static NetworkTableEntry entryYaw = myCam.getEntry("yaw");
    private static NetworkTableEntry entryPipeline = myCam.getEntry("pipeline");
    private static NetworkTableEntry entryDriverMode = myCam.getEntry("driver_mode");
    private static NetworkTableEntry entryIsValid = myCam.getEntry("is_valid");
    private static NetworkTableEntry NetworkTableTargetPose = myCam.getEntry("pose");

    private static double[] targetPose = NetworkTableTargetPose.getDoubleArray(new double[0]);

    private static double valueYaw;
    private static double valuePipeline = 0;
    private static boolean valueDriverMode = false;
    private static boolean valueIsValid;

    private static double[] ballPositions = NTballPositions.getDoubleArray(new double[0]);
    private static double[] ballSizes = NTballSizes.getDoubleArray(new double[0]);

    public static double getDistance() {
        if (targetPose.length > 0) {
            // return targetPose[1];
            return Math.hypot(Math.abs(metersToFeet(targetPose[0])), Math.abs(metersToFeet(targetPose[1])));
        } else {
            return -1.0;
        }
    }

    public static double metersToFeet(double meters) {
        return meters * 3.28084;
    }

    public static double getBallTurnValue() {
        int indexOfMax = 0;
        double maxSize = -1.0;
        if (ballSizes.length <= 0 || ballPositions.length <= 0) {
            return 0;
        }
        for (int i = 0; i < ballSizes.length; i++) {
            if (ballSizes[i] > maxSize) {
                maxSize = ballSizes[i];
                indexOfMax = i;
            }
        }
        DriverStation.reportError("maxSize " + maxSize, false);
        return (ballPositions[indexOfMax] - 160) / 640;
    }

    private static void updateGetValues() {
        valueYaw = entryYaw.getDouble(0.0);
        valueIsValid = entryIsValid.getBoolean(false);
    }

    private static void updateSetValue() {
        entryPipeline.setDouble(valuePipeline);
        entryDriverMode.setBoolean(valueDriverMode);
    }

    // #region getters

    public static double getYaw() {
        updateGetValues();
        return valueYaw;
    }

    public static boolean getIsValid() {
        updateGetValues();
        return valueIsValid;
    }

    // -30 --> 30
    public static double getTurnValue() {

        double yaw = getYaw();
        if (yaw > 15) {
            yaw = 15.0;
        } else if (yaw < -15) {
            yaw = -15.0;
        }
        double power = (yaw / 30.0) * .5;
        return power;
    }

    public static int getLedValue() {

        double yaw = Math.abs(getYaw());
        double percent = (yaw / 30.0) * 100;
        if (percent > 100) {
            percent = 100;
        }
        return (int) (105.0 - percent);
    }

    public static double getStrafeValue() {
        double strafeValue = getYaw() / 50;
        return strafeValue;
    }

    // #endregion

    // #region setters
    public static void setPipeline(double pipeline) {
        valuePipeline = pipeline;
        updateSetValue();
    }

    public static void setDriverMode(boolean driverMode) {
        valueDriverMode = driverMode;
        updateSetValue();
    }
    // #endregion
}