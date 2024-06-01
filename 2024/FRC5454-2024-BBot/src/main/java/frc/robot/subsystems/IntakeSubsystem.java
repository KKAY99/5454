package frc.robot.subsystems;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.classes.ObsidianCANSparkMax;

public class IntakeSubsystem extends SubsystemBase {
  private TalonSRX m_motor1;
  private TalonSRX m_motor2;

  public IntakeSubsystem(int motor1ID,int motor2ID) {
    m_motor1 = new TalonSRX (motor1ID);
    m_motor2 = new TalonSRX(motor2ID);
  }
  public void run(double power) {
    m_motor1.set(TalonSRXControlMode.PercentOutput, power);
    m_motor2.set(TalonSRXControlMode.PercentOutput,power);
  }
  public void stop() {
    m_motor1.set(TalonSRXControlMode.PercentOutput,0);
    m_motor2.set(TalonSRXControlMode.PercentOutput,0);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run

  }
}
