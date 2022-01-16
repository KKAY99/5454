package frc.robot.drive.swerve;

public class PidConstants {
    public double kP = 0;
    public double kI = 0;
    public double kD = 0;
    public double kF = 0;

    public PidConstants(double p, double i, double d) {
        kP = p;
        kI = i;
        kD = d;
    }

    public PidConstants(double p, double i, double d, double f) {
        kP = p;
        kI = i;
        kD = d;
        kF = f;
    }
}