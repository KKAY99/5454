package frc.robot.subsystems;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.classes.ObsidianCANSparkMax;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

public class ShooterSubsystem extends SubsystemBase {
  private ObsidianCANSparkMax m_motor1;
  private ObsidianCANSparkMax m_motor2;
  private TalonSRX m_inclineMotor1;
  private TalonSRX m_inclineMotor2;


  public ShooterSubsystem(int motor1ID,int motor2ID) {
    m_motor1 = new ObsidianCANSparkMax(motor1ID,MotorType.kBrushless,true);
    m_motor2 = new ObsidianCANSparkMax(motor2ID,MotorType.kBrushless,true);
  }
  public void runShooterMotors(double power) {
    m_motor1.set(power);
    m_motor2.set(power);
  }
  public void stopShooterMotors() {
    m_motor1.stopMotor();
    m_motor2.stopMotor();
  }
  public void runShooterIncline(double power) {
   m_inclineMotor1.set(TalonSRXControlMode.PercentOutput, power);
    m_inclineMotor2.set(TalonSRXControlMode.PercentOutput,power);
  }
  public void stopShooterIncline() {
    m_inclineMotor1.set(TalonSRXControlMode.PercentOutput, 0);
    m_inclineMotor2.set(TalonSRXControlMode.PercentOutput,0);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run

  }
}
