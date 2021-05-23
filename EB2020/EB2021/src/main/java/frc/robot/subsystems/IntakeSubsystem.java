package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.PWMVictorSPX;
import frc.robot.Constants.IntakeConstants;
public class IntakeSubsystem extends SubsystemBase {
    private PWMVictorSPX m_IntakeMotor;
    /**
     * Creates a new IntakeSubsystem.
     */
 
    public IntakeSubsystem() {
        m_IntakeMotor=new PWMVictorSPX(IntakeConstants.intakeMotorPort);
    }
  
  public void setSpeed(double speed)
  {
     m_IntakeMotor.set(speed);
  
  }
  
    @Override
    public void periodic() {
      // This method will be called once per scheduler run
    }
  }
  