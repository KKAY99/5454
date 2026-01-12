package frc.robot.utilities;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

/** Add your docs here. */
public class photonvision{
    public NetworkTableEntry m_x;
    public NetworkTableEntry m_y;

    public photonvision(String cameraName){
        NetworkTable Table = NetworkTableInstance.getDefault().getTable(cameraName);
        m_x =Table.getEntry("x");
        m_y =Table.getEntry("y");
    }

    public double getX() {
        return m_x.getDouble(0.0);
    }
    public double getY() {
        return m_y.getDouble(0.0);
    }

}

