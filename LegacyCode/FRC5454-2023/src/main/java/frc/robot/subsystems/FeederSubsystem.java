package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class FeederSubsystem extends SubsystemBase {
    CANSparkMax m_FeederMotor;

    /** Creates a new ExampleSubsystem. */
    public FeederSubsystem(Integer FeederPort) {
        m_FeederMotor = new CANSparkMax(FeederPort, MotorType.kBrushless);
        m_FeederMotor.setOpenLoopRampRate(0.25);
    }

    public void run(double speed) {
        m_FeederMotor.set(speed);
    }
   

    public void stop() {
        m_FeederMotor.set(0.0);
    }
    public double getFeederSpeed()
    {
        return m_FeederMotor.get();
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }
}
