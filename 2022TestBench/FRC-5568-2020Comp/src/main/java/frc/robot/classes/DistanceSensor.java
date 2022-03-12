package frc.robot.classes;

import com.revrobotics.Rev2mDistanceSensor;
import com.revrobotics.Rev2mDistanceSensor.Unit;
import com.revrobotics.Rev2mDistanceSensor.Port;

public class DistanceSensor {
    public static Character feet = 'F';
    public static Character inches = 'I';
    public static Character millimeters = 'm';
    public static Character meters = 'M';

    private Rev2mDistanceSensor m_distanceSensor;

    public DistanceSensor(Rev2mDistanceSensor distanceSensor) {
        m_distanceSensor = distanceSensor;
        m_distanceSensor = new Rev2mDistanceSensor(Port.kOnboard);
        
    }

    public double getDist(char unit) {
        // if (m_distanceSensor.isRangeValid()) {
        switch (unit) {
        case 'F':
            return m_distanceSensor.getRange(Unit.kInches) / 12.0;

        case 'I':
            return m_distanceSensor.getRange(Unit.kInches);

        case 'M':
            return m_distanceSensor.getRange(Unit.kMillimeters) * 0.001;

        case 'm':
            return m_distanceSensor.getRange(Unit.kMillimeters);

        default:
            return m_distanceSensor.getRange(Unit.kInches);
        }
        // } else {
        // return -1.0;
        // }

    }
}