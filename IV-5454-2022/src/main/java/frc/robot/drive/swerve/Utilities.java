package frc.robot.drive.swerve;


public class Utilities {
    public static double[] GetPolarFromCartesian(double xMag, double yMag) {
        // Joystick magnitude
        double controllerMagnitude = Math.sqrt(Math.pow(xMag, 2) + Math.pow(yMag, 2));

        // Joystick degrees
        double initialDegrees = (Math.atan2(yMag, xMag) * (180 / Math.PI)) + 90;
        double controllerDegrees = 360 - initialDegrees;
        controllerDegrees = controllerDegrees > 360 
            ? controllerDegrees - 360 
            : controllerDegrees;
        
        // Flip direction that degrees increases
        controllerDegrees = -controllerDegrees + 360;

        // Deadband
        controllerMagnitude = Deadband(controllerMagnitude, 0.1);
        if (controllerMagnitude == 0) {
            controllerDegrees = 0;
        }

        return new double[] { controllerMagnitude, controllerDegrees };
    }

    public static double Deadband(double input, double deadband) {
        if (Math.abs(input) > deadband) {
            return input;
        } else {
            return 0;
        }
    }

    public static double ClampAngle(double angle) {
        double finalAngle = 0;

        if (angle >= 180) {
            finalAngle = (angle % 360) - 360;
        } else if (angle < -180) {
            finalAngle = 360 - (angle % 360);
        } else {
            finalAngle = angle;
        }

        return finalAngle;
    }

    public static double ClampContinuousAngle(double currentReading) {
        double finalAngle = 0;

        if (currentReading > 360) {
            finalAngle = currentReading % 360;
        } else if (currentReading < 0) {
            finalAngle = 360 - (Math.abs(currentReading) % 360);
        } else {
            finalAngle = currentReading;
        }

        return finalAngle;
    }
}