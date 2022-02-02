package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.motorcontrol.PWMVictorSPX;
import frc.robot.Constants.ClimberConstants;
public class ClimberSubsystem extends SubsystemBase {
    private PWMVictorSPX m_LiftMotor;
    /**
     * Creates a new IntakeSubsystem.
     */
 
    public ClimberSubsystem() {
        m_LiftMotor=new PWMVictorSPX(ClimberConstants.climberMotorPort);
    }
    public void setSpeed(double speed){
        m_LiftMotor.set(speed);
    }
  
    @Override
    public void periodic() {
      // This method will be called once per scheduler run
    }
}
