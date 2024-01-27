package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;    
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.robot.utilities.Limelight;
import frc.robot.utilities.ShotTable;
public class ShooterSubsystem extends SubsystemBase{
    private Limelight m_limeLight;

    private CANSparkMax m_ShootingMotor1;
    private CANSparkMax m_ShootingMotor2;

    private SparkMaxPIDController m_pidController1;
    private SparkMaxPIDController m_pidController2;

    private double maxRPM;
    private double m_distance;

    public ShooterSubsystem(Limelight limeLight,int shootingMotor1,int shootingMotor2){
        //TEST PLACE
        m_limeLight=limeLight;
 
/*         power=shotTable.getVelocity(distance);
        System.out.println("distance: " + distance + " -- " + "Power:" + power);
           power=shotTable.getVelocity(distance);
        distance=5;
           System.out.println("distance: " + distance +  " -- " + "Power:" + power);
       distance=21;
           power=shotTable.getVelocity(distance);
        System.out.println("distance: " + distance  + " -- " + "Power:" + power);
        distance=34;
        power=shotTable.getVelocity(distance);
        System.out.println("distance: " + distance  + " -- " + "Power:" + power);
        distance=43;
        
        power=shotTable.getVelocity(distance);
        System.out.println("distance: " + distance + " -- "+ "Power:" + power);
        distance=53;
        power=shotTable.getVelocity(distance);
        System.out.println("distance: " + distance + " -- "+ "Power:" + power);
        distance=211;
        power=shotTable.getVelocity(distance);
        System.out.println("distance: " + distance + " -- "+ "Power:" + power);*/
        
        m_ShootingMotor1=new CANSparkMax(shootingMotor2,MotorType.kBrushless);  
        m_ShootingMotor2=new CANSparkMax(shootingMotor1,MotorType.kBrushless);

        m_pidController1 = m_ShootingMotor1.getPIDController();
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
}
