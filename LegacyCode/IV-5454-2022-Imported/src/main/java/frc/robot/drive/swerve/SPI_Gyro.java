package frc.robot.drive.swerve;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;

public class SPI_Gyro extends ADXRS450_Gyro {
    public double GetAdjustedAngle() {
        double currentReading = this.getAngle();
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