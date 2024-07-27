package frc.robot.subsystems;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.classes.ObsidianCANSparkMax;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;

public class ShooterSubsystem extends SubsystemBase {
  private ObsidianCANSparkMax m_motor1;
  private ObsidianCANSparkMax m_motor2;
  private ObsidianCANSparkMax m_inclineMotor;
  private VictorSP m_feederMotor;

  private AbsoluteEncoder m_absEncoder;


  public ShooterSubsystem(int motor1ID,int motor2ID, int inclineMotorID, int feederMotorID) {
    m_motor1 = new ObsidianCANSparkMax(motor1ID,MotorType.kBrushless,true);
    m_motor2 = new ObsidianCANSparkMax(motor2ID,MotorType.kBrushless,true);
    m_inclineMotor = new ObsidianCANSparkMax(inclineMotorID, MotorType.kBrushed, true);
    m_feederMotor=new VictorSP(feederMotorID);

    m_absEncoder=m_inclineMotor.getAbsoluteEncoder();
  }

  public void runFeeder(double power) {
   m_feederMotor.set(power);
  }

  public void stopFeeder() {
    m_feederMotor.set(0);
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
   m_inclineMotor.set(power);
  }

  public void stopShooterIncline() {
    m_inclineMotor.stopMotor();
  }

  public double getEncoderValue(){
    return m_absEncoder.getPosition();
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run

  }
}
