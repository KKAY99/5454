package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class TurretSubsystem extends SubsystemBase{
  private CANSparkMax m_turretMotor;

  public TurretSubsystem(int turretMotorPort){
    m_turretMotor=new CANSparkMax(turretMotorPort,MotorType.kBrushless);
  }

  public void TrackTarget(boolean bool){

  }

  public void RunTurretMotor(double power){
    m_turretMotor.set(power);
  }

  public void stop(){
    m_turretMotor.stopMotor();
  }

  @Override
  public void periodic(){
  }
}
