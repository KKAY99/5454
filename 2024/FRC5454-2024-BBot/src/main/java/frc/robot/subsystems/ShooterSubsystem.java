package frc.robot.subsystems;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.AlternateEncoderType;
import com.revrobotics.EncoderType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.classes.ObsidianCANSparkMax;
import pabeles.concurrency.ConcurrencyOps.Reset;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder;

import edu.wpi.first.wpilibj.Encoder;

public class ShooterSubsystem extends SubsystemBase {
  private ObsidianCANSparkMax m_motor1;
  private ObsidianCANSparkMax m_motor2;
  private ObsidianCANSparkMax m_inclineMotor;
  private VictorSP m_feederMotor;
  private double kAngledeadband = 0.05;
  private SparkPIDController m_pidController;
  private RelativeEncoder m_inclineEncoder;
  private double kkMotor1Modifier=1.0;
  private double kMotorDeadBand=400; // was 200

  public ShooterSubsystem(int motor1ID,int motor2ID, int inclineMotorID, int feederMotorID) {
    m_motor1 = new ObsidianCANSparkMax(motor1ID,MotorType.kBrushless,true);
    m_motor2 = new ObsidianCANSparkMax(motor2ID,MotorType.kBrushless,true);
    m_motor1.setSmartCurrentLimit(Constants.k30Amp);
    m_motor2.setSmartCurrentLimit(Constants.k30Amp);
    m_inclineMotor = new ObsidianCANSparkMax(inclineMotorID, MotorType.kBrushed, true);
    m_feederMotor=new VictorSP(feederMotorID);

    m_inclineEncoder=m_inclineMotor.getAlternateEncoder(AlternateEncoderType.kQuadrature,8192);
    m_pidController=m_inclineMotor.getPIDController();
    m_pidController.setFeedbackDevice(m_inclineEncoder);

    //TODO: Needs more tuning overshoots by around 2.5 
    m_pidController.setP(2.5);
    m_pidController.setI(0.0018);
    m_pidController.setD(0.0015);
    m_pidController.setOutputRange(-1,1);
    
  }

  public void runFeeder(double power) {
   m_feederMotor.set(power);
  }

  public void stopFeeder() {
    m_feederMotor.set(0);
  }

  public void runShooterMotors(double power) {
    //offsetting power by constant due to different force on motors
    m_motor1.set(power*kkMotor1Modifier);
    m_motor2.set(power);
  }
  public void stopShooterMotors() {
    m_motor1.stopMotor();
    m_motor2.stopMotor();
  }

  public double getMotor1Veloc(){
    return m_motor1.getEncoder().getVelocity();
  }

  public double getMotor2Veloc(){
    return m_motor2.getEncoder().getVelocity();
  }

  public boolean isAtVeloc(double power){
    boolean returnValue=false;
    double desiredVeloc1=Math.abs(power*kkMotor1Modifier*5676); // was 56760
    double desiredVeloc2=Math.abs(power*5676); // was 56760
    desiredVeloc1=desiredVeloc1*.95; //FRICTION
    desiredVeloc2=desiredVeloc2*.95; //FRICTION
    if(Math.abs(getMotor1Veloc())-kMotorDeadBand<desiredVeloc1&&Math.abs(getMotor1Veloc())+kMotorDeadBand>desiredVeloc1&&
      Math.abs(getMotor2Veloc())-kMotorDeadBand<desiredVeloc2&&Math.abs(getMotor2Veloc())+kMotorDeadBand>desiredVeloc2){
        returnValue=true;
    }
    System.out.println("Desired Veloc:"+ desiredVeloc1 + "/" + desiredVeloc2);
    System.out.println("Motor1Veloc "+getMotor1Veloc());
    System.out.println("Motor2Veloc "+getMotor2Veloc());
    return returnValue;
  }

  public void runShooterIncline(double power) {
   m_inclineMotor.set(power);
  }

  public void stopShooterIncline() {
    m_inclineMotor.stopMotor();
  }

  public double getInclineEncoderValue(){
    return m_inclineEncoder.getPosition();
  }

  public boolean isAtAngleWithDeadband(double targetAngle){
    //use absolute value of angle
    double currentAngle = Math.abs(getInclineEncoderValue());
    targetAngle=Math.abs(targetAngle);
    if((currentAngle > targetAngle - kAngledeadband) && (currentAngle < targetAngle + kAngledeadband)){
      return true;
    }else{
      return false;
    }
  }

  public void SetReference(double position){
    ResetReference();
    m_pidController.setReference(position, ControlType.kPosition);
    System.out.println("TargetPos + "+position + "CurrentPos + "+ getInclineEncoderValue());


  }

  public void ResetReference(){
    m_inclineMotor.set(0);
    m_pidController.setReference(0, ControlType.kVelocity);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

  }
}
