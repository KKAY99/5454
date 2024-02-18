package frc.robot.utilities;
import org.littletonrobotics.junction.AutoLog;


public interface LasercanIO {
    @AutoLog
    public class LasercanIOInputs{
        public boolean lowTurretBreakBeam=false;
        public boolean highTurretBreakBeam=false;
    }

    public default void updateInputs(LasercanIOInputs inputs){}
}
