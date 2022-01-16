package frc.robot.drive.swerve;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class LimeLight {
    public NetworkTable NetworkTable;

    public LimeLight() {
        NetworkTable = NetworkTableInstance.getDefault().getTable("limelight");
    }

    public double GetTargetDelta() {
        return NetworkTable.getEntry("tx").getNumber(Integer.MIN_VALUE).doubleValue();
    }

    public double GetTargetArea() {
        return NetworkTable.getEntry("ta").getNumber(Integer.MIN_VALUE).doubleValue();
    }

    public boolean GetTargetSeen() {
        return NetworkTable.getEntry("tv").getNumber(Integer.MIN_VALUE).intValue() == 1;
    }
}
