package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.motorcontrol.Talon;

public class DriveBase extends SubsystemBase{
    private Talon m_LeftMotor1;
    private Talon m_LeftMotor2;
    private Talon m_RightMotor1;
    private Talon m_RightMotor2;

    public DriveBase(int leftMotor1Port,int leftMotor2Port,int rightMotor1Port,int rightMotor2Port){
        m_LeftMotor1=new Talon(leftMotor1Port);
        m_LeftMotor2=new Talon(leftMotor2Port);
        m_RightMotor1=new Talon(rightMotor1Port);
        m_RightMotor2=new Talon(rightMotor2Port);

        m_LeftMotor1.addFollower(m_LeftMotor2);
        m_RightMotor1.addFollower(m_RightMotor2);
    }

    public void RunLeftMotors(double speed){
        m_LeftMotor1.set(speed);
    }

    public void RunRightMotors(double speed){
        m_RightMotor2.set(speed);
    }

    public void StopMotors(){
        m_LeftMotor1.stopMotor();
        m_RightMotor1.stopMotor();
    }

}
