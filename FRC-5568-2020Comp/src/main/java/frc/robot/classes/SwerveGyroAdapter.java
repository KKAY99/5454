package frc.robot.classes;

//Shamelessly stolen from 1540 the Flaming Chickens. They offered it to us though, so it's ok.
import edu.wpi.first.wpilibj.GyroBase;

public class SwerveGyroAdapter extends GyroBase {

    private double angle = 0;

    @Override
    public void calibrate() {

    }

    @Override
    public void reset() {

    }

    public void setAngle(double angle) {
        this.angle = angle;
    }

    @Override
    public double getAngle() {
        return angle;
    }

    @Override
    public double getRate() {
        return 0;
    }

    public void close() {

    }

}