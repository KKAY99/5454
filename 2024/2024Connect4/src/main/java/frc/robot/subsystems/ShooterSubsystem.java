package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class ShooterSubsystem extends SubsystemBase{
    //private TalonSRX m_ShootingMotor1;
    //private TalonSRX m_ShootingMotor2;
    //private CANSparkMax m_ShootingMotor1;
    //private CANSparkMax m_ShootingMotor2;
    private WPI_TalonFX m_ShootingMotor1;
    private WPI_TalonFX m_ShootingMotor2;
    private CANSparkMax m_feederMotor;
    private SparkMaxPIDController m_pidController1;
    private SparkMaxPIDController m_pidController2;

    private double maxRPM;

    public ShooterSubsystem(int shootingMotor1,int shootingMotor2,int feedMotor){
        //m_ShootingMotor1=new TalonSRX(shootingMotor1);  
        //m_ShootingMotor2=new TalonSRX(shootingMotor2);
        //m_ShootingMotor1=new CANSparkMax(shootingMotor2,MotorType.kBrushless);  
        //m_ShootingMotor2=new CANSparkMax(shootingMotor1,MotorType.kBrushless);
        m_ShootingMotor1=new WPI_TalonFX(shootingMotor1);  
        m_ShootingMotor2=new WPI_TalonFX(shootingMotor2);
        configmotor(m_ShootingMotor1);
        configmotor(m_ShootingMotor2);
        m_feederMotor=new CANSparkMax(feedMotor, MotorType.kBrushless);
        /*m_pidController1 = m_ShootingMotor1.getPIDController();
        m_pidController2 = m_ShootingMotor2.getPIDController();

        double kP = 6e-5; 
        double kI = 0;
        double kD = 0; 
        double kIz = 0; 
        double kFF = 0.000015; 
        double kMaxOutput = 1; 
        double  kMinOutput = -1;
        maxRPM = 5676;

        m_pidController1.setP(kP);
        m_pidController1.setI(kI);
        m_pidController1.setD(kD);
        m_pidController1.setIZone(kIz);
        m_pidController1.setFF(kFF);
        m_pidController1.setOutputRange(kMinOutput, kMaxOutput);

        m_pidController2.setP(kP);
        m_pidController2.setI(kI);
        m_pidController2.setD(kD);
        m_pidController2.setIZone(kIz);
        m_pidController2.setFF(kFF);
        m_pidController2.setOutputRange(kMinOutput, kMaxOutput);
        */
    }

 public void configmotor(WPI_TalonFX motor){
   motor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor,
                                        0,30);
   motor.configNominalOutputForward(0,30);
   motor.configNominalOutputReverse(0,30);
   motor.configPeakOutputForward(1,30);
   motor.configPeakOutputReverse(-1,30);
   motor.config_kF(0, 1023/2066.0,30);
   motor.config_kP(0, .1,30);
   motor.config_kI(0, 0.001,30);
   motor.config_kD(0, 5,30);
   
 }
 public void RunTopMotor(double speed){
  //      m_ShootingMotor1.set(ControlMode.PercentOutput,speed);
   
       }
 public void RunShootingMotors(double speed){
        //m_ShootingMotor1.set(speed);    
        //m_ShootingMotor2.set(speed);
        //speed=speed*maxRPM;
//        m_pidController1.setReference(speed,CANSparkMax.ControlType.kVelocity);
//        m_pidController2.setReference(speed,CANSparkMax.ControlType.kVelocity);
        m_ShootingMotor1.set(ControlMode.Velocity, speed);
        m_ShootingMotor2.set(ControlMode.Velocity, speed);
        m_feederMotor.set(ShooterConstants.feederSpeed);
      //  speed=0.88;
       // System.out.println("overriding shooter to set percentge - " + speed);
      // m_ShootingMotor1.set(ControlMode.PercentOutput,speed);
      //  m_ShootingMotor2.set(ControlMode.PercentOutput, speed);   
}  

    public void StopShootingMotors(){
        System.out.print("stop Motor Subsysem");
        m_ShootingMotor1.set(0);
        m_ShootingMotor2.set(0);
        m_feederMotor.set(0);
    }
}
