package frc.robot.classes.utilfuncs;

/**
 * This class is used to run swerve drive.
 */
public class UtilFuncs {
    public static boolean isOutOfDeadzone(double inputVariable, double deadzone) {
        return Math.abs(inputVariable) > Math.abs(deadzone);
    }

    public static double motorPowerClip(double power) {
        if (power > 1) {
            power = 1;
        } else if (power < -1) {
            power = -1;
        }
        return power;
    }

    public static boolean aboutEqual(double value, double maxRange) {
        return isOutOfDeadzone(value, maxRange);
    }

    public static double deadzoneModify(double value, double deadzone) {
        if (isOutOfDeadzone(value, deadzone))
            return value;
        else
            return 0.0;
    }
}
