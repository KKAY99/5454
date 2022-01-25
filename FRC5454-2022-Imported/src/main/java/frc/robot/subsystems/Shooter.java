package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
  TalonFX Bottom_ShooterMotor_One;
  TalonFX Bottom_ShooterMotor_Two;
  CANSparkMax Top_ShooterMotor_One;

  /** Creates a new ExampleSubsystem. */
  public Shooter(Integer BottomOnePort, Integer BottomTwoPort, Integer TopOnePort) {
    Bottom_ShooterMotor_One = new TalonFX(BottomOnePort);
    Bottom_ShooterMotor_Two = new TalonFX(BottomTwoPort);
    Top_ShooterMotor_One = new CANSparkMax(TopOnePort, MotorType.kBrushless);
  }

  public void shoot(double topPower, double bottomPower) {
    Bottom_ShooterMotor_One.set(ControlMode.PercentOutput, bottomPower);
    Bottom_ShooterMotor_Two.set(ControlMode.PercentOutput, bottomPower);
    Top_ShooterMotor_One.set(topPower);
  }

  public void stop(){
    Bottom_ShooterMotor_One.set(ControlMode.PercentOutput, 0.0);
    Bottom_ShooterMotor_Two.set(ControlMode.PercentOutput, 0.0);
    Top_ShooterMotor_One.set(0.0); 
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
