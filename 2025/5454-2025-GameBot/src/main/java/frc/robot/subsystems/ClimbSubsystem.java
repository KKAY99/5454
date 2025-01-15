package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utilities.ObsidianCANSparkMax;

public class ClimbSubsystem extends SubsystemBase {
  private ObsidianCANSparkMax m_motor;

  public ClimbSubsystem(int CanID){
    m_motor = new ObsidianCANSparkMax(CanID, MotorType.kBrushless, false);
  }

  public void run(double speed){
    m_motor.set(speed);
  }

  public void stop(){
    m_motor.stopMotor();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
