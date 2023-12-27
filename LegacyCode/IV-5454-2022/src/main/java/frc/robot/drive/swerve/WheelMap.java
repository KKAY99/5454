package frc.robot.drive.swerve;

public class WheelMap {
    public int DriveMotorChannel = 0;
    public int RotatorMotorChannel = 0;
    public int EncoderChannel = 0;
    public double EncoderOffset = 0;

    public WheelMap(int driveMotorChannel, int rotatorMotorChannel, int encoderChannel, double encoderOffset) {
        DriveMotorChannel = driveMotorChannel;
        RotatorMotorChannel = rotatorMotorChannel;
        EncoderChannel = encoderChannel;
        EncoderOffset = encoderOffset;
    }
}