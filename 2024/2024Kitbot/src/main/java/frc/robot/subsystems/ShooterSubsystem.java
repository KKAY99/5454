package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;    
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.motorcontrol.PWMVictorSPX;

public class ShooterSubsystem extends SubsystemBase{
    private TalonSRX m_ShootingMotor1;
    private TalonSRX m_ShootingMotor2;
    private PWMVictorSPX m_ShootingMotor3;
    private PWMVictorSPX m_ShootingMotor4;

    public ShooterSubsystem(int shootingMotor1,int shootingMotor2,int shooterMotor3,int shooterMotor4){
        m_ShootingMotor1=new TalonSRX(shootingMotor1);  
        m_ShootingMotor2=new TalonSRX(shootingMotor2);
        m_ShootingMotor3=new PWMVictorSPX(shooterMotor3);
        m_ShootingMotor4=new PWMVictorSPX(shooterMotor4);
    }

    public void RunTopMotor(double speed){
   
    }

    public void RunShootingMotors(double speed){
        m_ShootingMotor1.set(ControlMode.PercentOutput,speed);   
        m_ShootingMotor2.set(ControlMode.PercentOutput,speed);
        m_ShootingMotor3.set(-speed);   
        m_ShootingMotor4.set(-speed);
    }

    public void StopShootingMotors(){
        System.out.print("stop Motor Subsysem");
        m_ShootingMotor1.set(ControlMode.PercentOutput,0);
        m_ShootingMotor2.set(ControlMode.PercentOutput,0);
        m_ShootingMotor3.set(0);   
        m_ShootingMotor4.set(0);
    }
}
