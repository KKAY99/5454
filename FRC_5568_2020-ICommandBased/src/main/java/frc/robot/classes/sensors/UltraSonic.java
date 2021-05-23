package frc.robot.classes.sensors;

import edu.wpi.first.wpilibj.AnalogInput;

public class UltraSonic {
    private AnalogInput m_analogSensor;
    private final int m_countsToInch = 1;

    public UltraSonic(AnalogInput analogSensor) {
        m_analogSensor = analogSensor;
    }

    public double getOutput() {
        return m_analogSensor.getVoltage() * m_countsToInch;
    }
}