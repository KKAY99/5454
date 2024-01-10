package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.motorcontrol.Talon;

public class ShooterSubsystem extends SubsystemBase{
    private Talon m_ShootingMotor1;
    private Talon m_ShootingMotor2;

    public ShooterSubsystem(int shootingMotor1, int shootingMotor2){
        m_ShootingMotor1=new Talon(shootingMotor1);
        m_ShootingMotor2=new Talon(shootingMotor2);

        m_ShootingMotor1.addFollower(m_ShootingMotor2);
    }

    public void RunShootingMotors(double speed){
        m_ShootingMotor1.set(speed);
    }

    public void StopShootingMotors(){
        m_ShootingMotor1.stopMotor();
    }
}
