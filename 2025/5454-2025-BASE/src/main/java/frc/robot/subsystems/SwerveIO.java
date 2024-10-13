package frc.robot.subsystems;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public interface SwerveIO {
    @AutoLog
    public class SwerveIOInputs{
        public double frontLeftModuleSpeed=0.0;
        public double frontRightModuleSpeed=0.0;
        public double backLeftModuleSpeed=0.0;
        public double backRightModuleSpeed=0.0;
    }

    public default void updateInputs(SwerveIOInputs inputs){}
}
