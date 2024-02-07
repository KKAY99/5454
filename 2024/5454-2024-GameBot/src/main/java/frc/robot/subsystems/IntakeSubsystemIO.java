package frc.robot.subsystems;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public interface IntakeSubsystemIO {
    @AutoLog
    public class IntakeSubsystemIOInputs{
        public double intakeSpeed=0.0;
    }

    public default void updateInputs(IntakeSubsystemIOInputs inputs){}
}
