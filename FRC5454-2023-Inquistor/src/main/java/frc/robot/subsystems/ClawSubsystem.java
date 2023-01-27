package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class ClawSubsystem  extends SubsystemBase {
    CANSparkMax m_ClawMotor;
  
    /** Creates a new ExampleSubsystem. */
    public  ClawSubsystem(Integer MotorPort) {
      m_ClawMotor = new CANSparkMax(MotorPort, MotorType.kBrushed);   
      m_ClawMotor.setOpenLoopRampRate(0.25);
      m_ClawMotor.setSmartCurrentLimit(30);  // likely gets ignored due to brushed motor
      m_ClawMotor.setSecondaryCurrentLimit(30); //Set as well at 30
    }
    public void run(double power) {
      m_ClawMotor.set(power);
      
    }
  
    public void stop() {
      m_ClawMotor.set(0);
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
  