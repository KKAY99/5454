package frc.robot.drive.swerve;

import edu.wpi.first.wpilibj.DriverStation;

public class MultiLogger {
    public static void Debug(String message) {
        Debug(message, false);
    }

    public static void Debug(String message, boolean printDebugToConsole) {
        if (printDebugToConsole) {
            System.out.println(message);
        }
    }

    public static void Info(String message) {
        Info(message, false);
    }

    public static void Info(String message, boolean sendToDriverStation) {
        System.out.println(message);

        if (sendToDriverStation) {
            DriverStation.reportWarning(message, false);
        }
    }

    public static void Warning(String message) {
        Warning(message, null);
    }

    public static void Warning(String message, StackTraceElement[] stackTrace) {
        if (stackTrace == null) {
            DriverStation.reportWarning(message, false);
        } else {
            DriverStation.reportWarning(message, stackTrace);
        }
    }

    public static void Error(String message) {
        Error(message, null);
    }

    public static void Error(String message, StackTraceElement[] stackTrace) {
        if (stackTrace == null) {
            DriverStation.reportWarning(message, false);
        } else {
            DriverStation.reportWarning(message, stackTrace);
        }
    }
}
