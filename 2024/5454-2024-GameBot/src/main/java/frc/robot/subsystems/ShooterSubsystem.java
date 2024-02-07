package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;    
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.robot.utilities.Limelight;
import frc.robot.utilities.ShotTable;
public class ShooterSubsystem extends SubsystemBase{
    private Limelight m_limeLight;

    private CANSparkMax m_ShootingMotor1;
    private CANSparkMax m_ShootingMotor2;
    private CANSparkMax m_angleMotor;

    private SparkMaxPIDController m_pidController1;
    private SparkMaxPIDController m_pidController2;
    private SparkMaxPIDController m_anglePID;
     
    private double maxRPM;
    private double m_distance;

    public ShooterSubsystem(Limelight limeLight,int shootingMotor1,int shootingMotor2,int angleMotor){
        //TEST PLACE
        m_limeLight=limeLight;
        m_ShootingMotor1=new CANSparkMax(shootingMotor2,MotorType.kBrushless);  
        m_ShootingMotor2=new CANSparkMax(shootingMotor1,MotorType.kBrushless);
        m_angleMotor = new CANSparkMax(angleMotor,MotorType.kBrushless);
        m_anglePID = m_angleMotor.getPIDController();
        m_pidController1 = m_ShootingMotor1.getPIDController();
        m_pidController2 = m_ShootingMotor2.getPIDController();

        double kP = 6e-5; 
        double kI = 0;
        double kD = 0; 
        double kIz = 0; 
        double kFF = 0.000015; 
        double kMaxOutput = 1; 
        double  kMinOutput = -1;
    

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
    
        double anglekP = 6e-5; 
        double anglekI = 0;
        double anglekD = 0; 
        double anglekIz = 0; 
        double anglekFF = 0.000015; 
        double anglekMaxOutput = 1; 
        double anglekMinOutput = -1;
    
        m_anglePID.setP(anglekP);
        m_anglePID.setI(anglekI);
        m_anglePID.setD(anglekD);
        m_anglePID.setIZone(anglekIz);
        m_anglePID.setFF(anglekFF);
        m_anglePID.setOutputRange(anglekMinOutput, anglekMaxOutput);

    }
    
    public void RunShootingMotors(double speed){
        speed=speed*maxRPM;
        m_pidController1.setReference(speed,CANSparkMax.ControlType.kVelocity);
        m_pidController2.setReference(speed,CANSparkMax.ControlType.kVelocity);
    }

    public void StopShootingMotors(){
        System.out.print("stop Motor Subsysem");
        m_ShootingMotor1.set(0);
        m_ShootingMotor2.set(0);
    }

    public void OutPutDistance(){
        ShotTable shotTable = new ShotTable();
        double calculation=shotTable.getVelocity(m_limeLight.getDistance());
        System.out.println("Distance Calucations: "+calculation);
    }

    public void setAngle(double targetAngle){
        //FIX: TO DO SetAngle
    }
    public double getAngle(){
        //FIX: TO DO GetAngle
        return 999;
    }

    public void setBrakeOn(){
      m_ShootingMotor1.setIdleMode(IdleMode.kBrake);  
      m_ShootingMotor2.setIdleMode(IdleMode.kBrake);
    } 
  public void setCoastOn(){
         m_ShootingMotor1.setIdleMode(IdleMode.kCoast);
         m_ShootingMotor2.setIdleMode(IdleMode.kCoast);    
    
    }


}
