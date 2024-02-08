package frc.robot.subsystems;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public interface TurretSubsystemIO {
    @AutoLog
    public class TurretSubsystemIOInputs{
        public double turretSpeed=0.0;
        public double turretEncoder=0.0;
        public boolean turretLimit=false;
    }

    public default void updateInputs(TurretSubsystemIOInputs inputs){}
}
