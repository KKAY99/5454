package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.io.Console;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import frc.robot.Constants;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.utilities.Limelight;
import frc.robot.utilities.ShotTable;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ShooterSubsystem extends SubsystemBase{
    private Limelight m_limeLight;

    private WPI_TalonFX m_ShootingMotor1;
    private WPI_TalonFX m_ShootingMotor2;
    
    private CANSparkMax m_feederMotor;
    //private CANSparkMax m_angleMotor;

    //private SparkMaxPIDController m_anglePID;

    //private RelativeEncoder m_angleEncoder;
     
    private double maxRPM;
    private double m_velocity;
    private double m_distance;
    private double m_shotsTaken;

    public ShooterSubsystem(Limelight limeLight,int shootingMotor1,int shootingMotor2,int angleMotor,int feedMotor){
        m_limeLight=limeLight;
        m_ShootingMotor1=new WPI_TalonFX(shootingMotor1);  
        m_ShootingMotor2=new WPI_TalonFX(shootingMotor2);
        configmotor(m_ShootingMotor1);
        configmotor(m_ShootingMotor2);
        m_feederMotor=new CANSparkMax(feedMotor,MotorType.kBrushless);
        m_feederMotor.setSmartCurrentLimit(Constants.k30Amp);
        //m_angleMotor = new CANSparkMax(angleMotor,MotorType.kBrushless);
        //m_angleMotor.setSmartCurrentLimit(Constants.k30Amp);
        //m_anglePID = m_angleMotor.getPIDController();
        //m_angleEncoder=m_angleMotor.getEncoder();
    
        double anglekP = 6e-5; 
        double anglekI = 0;
        double anglekD = 0; 
        double anglekIz = 0; 
        double anglekFF = 0.000015; 
        double anglekMaxOutput = 1; 
        double anglekMinOutput = -1;
    
        /*m_anglePID.setP(anglekP);
        m_anglePID.setI(anglekI);
        m_anglePID.setD(anglekD);
        m_anglePID.setIZone(anglekIz);
        m_anglePID.setFF(anglekFF);
        m_anglePID.setOutputRange(anglekMinOutput, anglekMaxOutput);
        m_angleMotor.getEncoder();*/
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
    
    public void RunShootingMotors(double speed){
        //speed=speed*maxRPM;
        // m_ShootingMotor1.set(ControlMode.Velocity, speed);
       // m_ShootingMotor2.set(ControlMode.Velocity, speed);
       m_ShootingMotor1.set(ControlMode.PercentOutput, 1); 
       m_ShootingMotor2.set(ControlMode.PercentOutput, 1); 
       m_feederMotor.set(ShooterConstants.feederSpeed);
    }  

    public void StopShootingMotors(){
        m_ShootingMotor1.set(0);
        m_ShootingMotor2.set(0);
        m_feederMotor.set(0);
    }

    public boolean isMotorVelocityAtBase(){
        boolean returnValue=false;

        if(GetVelocityMotor1()+ShooterConstants.baseSpeedDeadband>=ShooterConstants.baseMotorSpeed*maxRPM&&GetVelocityMotor1()-ShooterConstants.baseSpeedDeadband<=ShooterConstants.baseMotorSpeed*maxRPM
            &&GetVelocityMotor2()+ShooterConstants.baseSpeedDeadband<=ShooterConstants.baseMotorSpeed*maxRPM&&GetVelocityMotor2()-ShooterConstants.baseSpeedDeadband<=ShooterConstants.baseMotorSpeed*maxRPM){
            returnValue=true;
        }

        return returnValue;
    }

    public double GetVelocityMotor1(){
        return m_ShootingMotor1.getSelectedSensorVelocity();
    }

    public double GetVelocityMotor2(){
        return m_ShootingMotor2.getSelectedSensorVelocity();
    }

    public void OutPutDistance(){
        ShotTable shotTable = new ShotTable();
        double calculation=shotTable.getVelocity(m_limeLight.getDistance());
        System.out.println("Distance Calucations: "+calculation);
    }

    public void RotateShooter(double speed){
        //m_angleMotor.set(speed);
    }

    public void stopRotate(){
        //m_angleMotor.set(0);
    }

    public void setAngle(double targetAngle){
        //m_anglePID.setReference(targetAngle,ControlType.kPosition);
    }

    public double getAngle(){
        return 0;//m_angleEncoder.getPosition();
    }

    public void ResetControlType(){
        //m_anglePID.setReference(0,ControlType.kVelocity);
    }

    public void setBrakeOn(){
        m_ShootingMotor1.setNeutralMode(NeutralMode.Brake);
        m_ShootingMotor1.setNeutralMode(NeutralMode.Brake);
        //m_angleMotor.setIdleMode(IdleMode.kBrake);
    } 
  
    public void setCoastOn(){
        m_ShootingMotor1.setNeutralMode(NeutralMode.Coast);
        m_ShootingMotor1.setNeutralMode(NeutralMode.Coast);
        //m_angleMotor.setIdleMode(IdleMode.kCoast);
    }

    public void ResetShotsTaken(){
        m_shotsTaken=0;
    }

    public void ShotTaken(){
        m_shotsTaken+=1;
    }

    public void PutTableMultiplier(){
    }

    @Override
    public void periodic(){
        Logger.recordOutput("Shooter/ShooterVelocity",m_velocity);
        Logger.recordOutput("Shooter/ShootMotor1Velocity",GetVelocityMotor1());
        Logger.recordOutput("Shooter/ShootMotor2Velocity",GetVelocityMotor2());
        Logger.recordOutput("Shooter/ShotsTaken",m_shotsTaken);
    }
}
