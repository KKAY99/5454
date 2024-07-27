package frc.robot.subsystems;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.classes.ObsidianCANSparkMax;

public class IntakeSubsystem extends SubsystemBase {
  private TalonSRX m_leftMotor;
  private TalonSRX m_rightMotor;

  public IntakeSubsystem(int leftMotorID,int rightMotorID) {
    m_leftMotor = new TalonSRX(leftMotorID);
    m_rightMotor = new TalonSRX(rightMotorID);
  }
  public void run(double power) {
    m_leftMotor.set(TalonSRXControlMode.PercentOutput, power);
    m_rightMotor.set(TalonSRXControlMode.PercentOutput,power);
  }
  public void stop() {
    m_leftMotor.set(TalonSRXControlMode.PercentOutput,0);
    m_rightMotor.set(TalonSRXControlMode.PercentOutput,0);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run

  }
}
