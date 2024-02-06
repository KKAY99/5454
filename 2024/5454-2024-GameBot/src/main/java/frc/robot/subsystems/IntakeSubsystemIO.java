package frc.robot.subsystems;
import org.littletonrobotics.junction.AutoLog;

public interface IntakeSubsystemIO {
    @AutoLog

    public static class IntakeSubsystemIOInputs{
        public double speed=0.0;
        public boolean lowBreakBeam=false;
        public boolean highBreakBeam=false;
    }

    public default void updateInputs(IntakeSubsystemIOInputs inputs){

    }

}
