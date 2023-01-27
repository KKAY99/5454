package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ColorWheel;
import edu.wpi.first.wpilibj.motorcontrol.PWMVictorSPX;
import frc.robot.Constants.ColorWheel;
public class ColorWheelSubsystem extends SubsystemBase {
private PWMVictorSPX m_WheelMotor;
    /**
     * Creates a new IntakeSubsystem.
     */
 
    public ColorWheelSubsystem() {
        m_WheelMotor=new PWMVictorSPX(ColorWheel.wheelMotorPort);
    }
  
  public void setSpeed(double speed)
  {
     m_WheelMotor.set(speed);
  
  }
  
    @Override
    public void periodic() {
      // This method will be called once per scheduler run
    }
  }
  