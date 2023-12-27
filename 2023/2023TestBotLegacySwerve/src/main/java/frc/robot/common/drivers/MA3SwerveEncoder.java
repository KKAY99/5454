package frc.robot.common.drivers;


import edu.wpi.first.wpilibj.AnalogInput;

/**
 * Encoders used in Invictus 3593's swerve modules.
 * 
 * @Model USDigital-MA3 analog 10-bit
 */
public class MA3SwerveEncoder extends AnalogInput {
    public static final double ENC_POSITIONS_PER_REVOLUTION = 4096d;
    public double BasePositionOffset = 0d;

    public MA3SwerveEncoder(int analogChannel, double basePositionOffset) {
        super(analogChannel);
        BasePositionOffset = basePositionOffset;
    }

    /**
     * Angle adjusted for base offset and clamped
     */
    public double GetRelativeAngle() {
        double offsetAngle = (GetOffsetPosition() / ENC_POSITIONS_PER_REVOLUTION) * 360;
        return ClampValue(offsetAngle, 0, 360);
    }

    /**
     * Position adjusted for base offset and clamped
     */
    public double GetRelativePosition() {
        double offsetPosition = GetOffsetPosition();
        return ClampValue(offsetPosition, 0, ENC_POSITIONS_PER_REVOLUTION);
    }

    /**
     * Angle adjusted for base offset
     */
    public double GetOffsetAngle() {
        return (GetOffsetPosition() / ENC_POSITIONS_PER_REVOLUTION) * 360;
    }

    /**
     * Position adjusted for base offset
     */
    public double GetOffsetPosition() {
        return this.getValue() - BasePositionOffset;
    }

    /**
     * Angle without adjustment
     */
    public double GetActualAngle() {
        return (GetActualPosition() / ENC_POSITIONS_PER_REVOLUTION) * 360;
    }

    /**
     * Position without adjustment
     */
    public double GetActualPosition() {
        return this.getValue();
    }

    public static double GetPositionFromAngle(double angle) {
        return (angle / 360) * ENC_POSITIONS_PER_REVOLUTION;
    }

    // Methods for extending PIDSource    
    public double pidGet() {
        return GetRelativeAngle() - 180;
    }

  
    private double ClampValue(double value, double minValue, double maxValue) {
        if (value < minValue) {
            return maxValue + value;
        } else {
            return value;
        }
    }
}