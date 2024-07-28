package frc.robot.subsystems;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class ShooterSubsystem extends SubsystemBase{
    //private TalonSRX m_ShootingMotor1;
    //private TalonSRX m_ShootingMotor2;
    private CANSparkMax m_ShootingMotor1;
    private CANSparkMax m_ShootingMotor2;
    //private WPI_TalonFX m_ShootingMotor1;
    //private WPI_TalonFX m_ShootingMotor2;
    private VictorSP m_feederMotor;
    private SparkMaxPIDController m_pidController1;
    private SparkMaxPIDController m_pidController2;
    private DigitalInput m_breakbeam;

    private double maxRPM;

    public ShooterSubsystem(int shootingMotor1,int shootingMotor2,int feedMotor,int sensorPort){
         m_ShootingMotor1=new CANSparkMax(shootingMotor2,MotorType.kBrushless);  
        m_ShootingMotor2=new CANSparkMax(shootingMotor1,MotorType.kBrushless);
        m_feederMotor=new VictorSP(feedMotor);
        m_breakbeam=new DigitalInput(sensorPort);

    }
 public void RunShootingMotors(double speed,double feederSpeed){
        System.out.println("Speed - " + speed);
        m_ShootingMotor1.set(-speed);    
        m_ShootingMotor2.set(speed);
        m_feederMotor.set(feederSpeed);

} 
public void RunShootingMotorsOnly (double speed){
        System.out.println("Shooter  Only Speed - " + speed);
        m_ShootingMotor1.set(-speed);    
        m_ShootingMotor2.set(speed);
       
} 
public void RunFeedingMotorOnly (double speed){
        m_feederMotor.set(speed);       
       
} 

    public void StopShootingMotors(){
        System.out.print("stop Motor Subsysem");
        m_ShootingMotor1.set(0);
        m_ShootingMotor2.set(0);
        m_feederMotor.set(0);
    }
}
