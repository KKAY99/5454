package frc.robot.subsystems;

import java.util.function.DoubleSupplier;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.CANSparkBase.ControlType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.DigitalInput;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAlternateEncoder.Type;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.CANCoderFaults;
import com.ctre.phoenix.sensors.CANCoderStickyFaults;
import com.ctre.phoenix.sensors.MagnetFieldStrength;
import com.ctre.phoenix.sensors.WPI_CANCoder;
public class TestSwerveModuleSubsystem extends SubsystemBase{
    private CANSparkMax m_turningMotor;
    private RelativeEncoder m_turningEncoder;
    private SparkMaxPIDController m_pid;
    private CANSparkMax m_driveMotor;
    private WPI_CANCoder m_encoder;
    public TestSwerveModuleSubsystem(int driveMotorPort,int turnMotorPort,int encoderPort){ 
        m_turningMotor = new CANSparkMax(turnMotorPort, MotorType.kBrushless);
        m_turningMotor.setSmartCurrentLimit(30);  
        m_turningMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        m_turningMotor.setInverted(true);
        m_turningEncoder=m_turningMotor.getEncoder();
        m_pid = m_turningMotor.getPIDController();
       // m_pid.setFeedbackDevice(
       // m_turningEncoder); // Configure feedback of the PID controller as the integrated encoder.
       m_driveMotor = new CANSparkMax(driveMotorPort, MotorType.kBrushless);
        m_driveMotor.setSmartCurrentLimit(30);  
        m_driveMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        m_encoder=new WPI_CANCoder(encoderPort,"5454Canivore");


        m_pid.setP(0.01, 1);
        m_pid.setI(0, 1);
        m_pid.setD(0, 1);
        m_pid.setFF(0, 1);
        m_pid.setIZone(0, 1);
        m_pid.setOutputRange(-1,1,1);

    }
    public void drive(double drive,double turn){
      driveMotor(drive);
      turn(turn);
    }
    public void driveMotor(double speed){
      m_driveMotor.set(speed);
    }
    public void turntoPos(double pos){
      System.out.println("Setting Pos to " + pos + " Current Pos: " + m_turningEncoder.getPosition());
      m_pid.setReference(pos, ControlType.kPosition,1);     
  
    }
 
    public void turn (double speed){
      m_turningMotor.set(speed);
    }
    
    public void stopDrive(){
      m_driveMotor.set(0);
    }
     public void stopTurn(){ 
      m_turningMotor.set(0);
    }

    public double getDriveSpeed(){
      return m_driveMotor.get();
    }
    public double getTurnSpeed(){
      return m_turningMotor.get();
    }
    public double getAngleCanCoder(){
      return 99;
    }
    public double getTurnEncoderPosition(){
      double posValue = m_encoder.getPosition();
      return posValue;
    }
    public double getTurnEncoderAbsolutePosition(){
      double absValue = m_encoder.getAbsolutePosition();  
      return absValue;
    }

    public double getTurnRelativeEncoderPosition(){
      double relativeValue = m_turningMotor.getEncoder().getPosition();
      return relativeValue;
    }

    public double getEncoderValue(){
      return m_encoder.getAbsolutePosition();
    }
    @Override
    public void periodic() {
//      System.out.println(getEncoderValue());
        // This method will be called once per scheduler run
//     System.out.println(" * Pos  " + getRotatePos() + " " + getLimitSwitch());
    }
}
