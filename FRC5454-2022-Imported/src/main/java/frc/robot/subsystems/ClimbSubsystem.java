package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax.IdleMode;

public class ClimbSubsystem extends SubsystemBase {
    private CANSparkMax m_ClimbMotor;
    private DigitalInput m_ClimbBottomLimitSwitch;
    private DigitalInput m_ClimbTopLimitSwitch;

    // private Counter m_ClimbLimitSwitchCounter;
    /** Creates a new ExampleSubsystem. */
    public ClimbSubsystem(Integer ClimbPort, Integer bottomLimitSwitchPort, Integer topLimitSwitchPort) {
        m_ClimbMotor = new CANSparkMax(ClimbPort, MotorType.kBrushed);
        m_ClimbMotor.setInverted(true);
        m_ClimbMotor.setIdleMode(IdleMode.kBrake);

        m_ClimbMotor.setOpenLoopRampRate(0.25);
        m_ClimbBottomLimitSwitch = new DigitalInput(bottomLimitSwitchPort);
        // m_ClimbLimitSwitchCounter=new Counter(m_ClimLimitSwitch);
        m_ClimbTopLimitSwitch = new DigitalInput(topLimitSwitchPort);

    }

    public boolean hitBottomLimit() {
        return (m_ClimbBottomLimitSwitch.get());
    }

    public boolean hitTopLimit() {
        return (m_ClimbTopLimitSwitch.get());
    }

    public void run(double speed) {
        System.out.println("setting climb speed=" + speed);
        m_ClimbMotor.set(speed);
    }

    public boolean stopForLimit(double speed) {
        if (((hitBottomLimit() == false) || (speed > 0)) && (hitTopLimit() == false || (speed < 0))) {
            return false;
        } else {
            return true;
        }
    }

    public void stop() {
        System.out.println("setting climb speed to 0");

        m_ClimbMotor.set(0.0);
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
