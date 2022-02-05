package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;

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


    Bottom_ShooterMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor,0,30); 
    Bottom_ShooterMotor.selectProfileSlot(0,0);
    Bottom_ShooterMotor.config_kP(0,0);
    Bottom_ShooterMotor.config_kI(0,0);
    Bottom_ShooterMotor.config_kD(0,0);
    Bottom_ShooterMotor.config_kF(0,.5);
    
    Top_ShooterMotor.setInverted(false);
    Top_ShooterMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor,0,30); 
    Top_ShooterMotor.selectProfileSlot(0,0);
    Top_ShooterMotor.config_kP(0,0);
    Top_ShooterMotor.config_kI(0,0);
    Top_ShooterMotor.config_kD(0,0);
    Top_ShooterMotor.config_kF(0,.5);
    
  }
  public double getTopMotorVelocity(){ 
    return Top_ShooterMotor.getSelectedSensorVelocity(0);
  }
  public double getBottomMotorVelocity(){
    return Top_ShooterMotor.getSelectedSensorVelocity(0);
  }
  public void shoot(double topVelocity, double bottomVelocity) {
    Bottom_ShooterMotor.set(ControlMode.Velocity, bottomVelocity);
    Top_ShooterMotor.set(ControlMode.Velocity, bottomVelocity);
 
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
