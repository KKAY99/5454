package frc.robot.utilities;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public interface LasercanIO {
    @AutoLog
    public class LasercanIOInputs{
        public boolean lowTurretBreakBeam=false;
        public boolean highTurretBreakBeam=false;
    }

    public default void updateInputs(LasercanIOInputs inputs){}
}
