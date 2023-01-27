package frc.robot.classes.sensors;

import edu.wpi.first.wpilibj.DigitalInput;

public class BinarySwitch {
    private DigitalInput di0;
    private DigitalInput di1;
    private DigitalInput di2;
    private DigitalInput di3;

    public BinarySwitch(int a, int b, int c, int d) {
        di0 = new DigitalInput(a);
        di1 = new DigitalInput(b);
        di2 = new DigitalInput(c);
        di3 = new DigitalInput(d);
    }

    public int getValue() {
        int cur = 0;

        // convert binary value to decimal
        if (!di0.get()) {
            cur += 1;
        }

        if (!di1.get()) {
            cur += 2;
        }

        if (!di2.get()) {
            cur += 4;
        }

        if (!di3.get()) {
            cur += 8;
        }

        return cur;
    }
}