package frc.robot.common.drivers;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;

import frc.robot.common.math.Rotation2;

public final class NavX extends Gyroscope {
    private final AHRS navX;

    public NavX(SPI.Port port) {
        this(port, (byte) 200);
    }

    public NavX(SPI.Port port, byte updateRate) {
        navX = new AHRS(port, updateRate);
    }

    @Override
    public void calibrate() {
        navX.reset();
    }

    @Override
    public Rotation2 getUnadjustedAngle() {
        return Rotation2.fromRadians(getAxis(Axis.YAW));
    }

    //KK Added 1/26/23
    public double getYaw() {
        return getAxis(Axis.YAW);
    }

    @Override
    public double getUnadjustedRate() {
        return Math.toRadians(navX.getRate());
    }

    public float getVelocityX(){
        return navX.getVelocityX();
    }
    public float getVelocityY(){
        return navX.getVelocityX();
    }
    public float getVelocityZ(){
        return navX.getVelocityX();
    }
    
    public boolean isMoving(float threshold){
        return ((getVelocityX()+getVelocityY()+getVelocityZ())>=threshold);
    }


    public double getAxis(Axis axis) {
        switch (axis) {
            case PITCH:
                return Math.toRadians(navX.getPitch());
            case ROLL:
                return Math.toRadians(navX.getRoll());
            case YAW:
                return Math.toRadians(navX.getYaw());
            default:
                return 0.0;
        }
    }

    public enum Axis {
        PITCH,
        ROLL,
        YAW
    }
}
