package frc.robot.subsystems;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimbSubsystem extends SubsystemBase {
    private CANSparkMax m_ClimbMotor;
    private DigitalInput m_ClimbLimitSwitch;
    private Counter m_ClimbLimitSwitchCounter;
    /** Creates a new ExampleSubsystem. */
    public ClimbSubsystem(Integer ClimbPort, Integer limitSwitchPort) {
        m_ClimbMotor = new CANSparkMax(ClimbPort, MotorType.kBrushed);   
        m_ClimbMotor.setInverted(false);
        m_ClimbMotor.setOpenLoopRampRate(0.25);
        m_ClimbLimitSwitch=new DigitalInput(limitSwitchPort);
        m_ClimbLimitSwitchCounter=new Counter(m_ClimbLimitSwitch);

    }
    public void resetlimitCounter()
    {
        m_ClimbLimitSwitchCounter.reset();
    }
    public int getLimitCounter()
    {
        return m_ClimbLimitSwitchCounter.get();
    }
    public boolean hitLimit()
    {
      return (m_ClimbLimitSwitch.get());
    }
    public void run(double speed) 
    {   System.out.println("setting climb speed=" + speed);
        m_ClimbMotor.set(speed);
    }

    

    public void stop() {
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
