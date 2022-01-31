package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {
  TalonFX Bottom_ShooterMotor;
  TalonFX Top_ShooterMotor;

  /** Creates a new ExampleSubsystem. */
  public ShooterSubsystem(Integer BottomPort, Integer TopPort) {
    Bottom_ShooterMotor = new TalonFX(BottomPort);
    Top_ShooterMotor = new TalonFX(TopPort);

    Bottom_ShooterMotor.configOpenloopRamp(0.5);
    Top_ShooterMotor.configOpenloopRamp(0.5);

    Bottom_ShooterMotor.setInverted(true);
  }

  public void shoot(double topPower, double bottomPower) {
    Bottom_ShooterMotor.set(ControlMode.PercentOutput, bottomPower);
    Top_ShooterMotor.set(ControlMode.PercentOutput, bottomPower);
  }

  public void stop() {
    Bottom_ShooterMotor.set(ControlMode.PercentOutput, 0.0);
    Top_ShooterMotor.set(ControlMode.PercentOutput, 0.0);
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
