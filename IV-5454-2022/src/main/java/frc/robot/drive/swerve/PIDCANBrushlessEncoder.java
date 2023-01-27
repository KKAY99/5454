package frc.robot.drive.swerve;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;

public class PIDCANBrushlessEncoder extends CANEncoder implements PIDSource {
    private PIDSourceType _pidSourcetype = PIDSourceType.kDisplacement;

    public PIDCANBrushlessEncoder(CANSparkMax device) {
        super(device);
    }

    @Override
    public void setPIDSourceType(PIDSourceType pidSource) {
        _pidSourcetype = pidSource;
    }

    @Override
    public PIDSourceType getPIDSourceType() {
        return _pidSourcetype;
    }

    @Override
    public double pidGet() {
        return _pidSourcetype == PIDSourceType.kDisplacement
            ? this.getPosition()
            : this.getVelocity();
    }

}