package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

public class ShooterSubsystem extends SubsystemBase{
    private TalonSRX m_ShootingMotor1;
    private TalonSRX m_ShootingMotor2;

    public ShooterSubsystem(int shootingMotor1, int shootingMotor2){
        m_ShootingMotor1=new TalonSRX(shootingMotor1);
        
        m_ShootingMotor2=new TalonSRX(shootingMotor2);
    }

    public void RunShootingMotors(double speed){
        m_ShootingMotor1.set(ControlMode.PercentOutput,speed);
        m_ShootingMotor2.set(ControlMode.PercentOutput,speed);
    }

    public void StopShootingMotors(){
        System.out.print("stop Motor Subsysem");
        m_ShootingMotor1.set(ControlMode.PercentOutput,0);
        m_ShootingMotor2.set(ControlMode.PercentOutput,0);
    }
}
