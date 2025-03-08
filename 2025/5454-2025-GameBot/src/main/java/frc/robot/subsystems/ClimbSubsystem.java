package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbConstants;
import frc.robot.Constants.DunkinDonutConstants;
import frc.robot.utilities.ObsidianCANSparkMax;
import frc.robot.utilities.ObsidianPID;

import com.ctre.phoenix6.configs.HardwareLimitSwitchConfigs;
import com.revrobotics.servohub.ServoHub.ResetMode;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkFlexExternalEncoder;
import edu.wpi.first.wpilibj.Servo;




public class ClimbSubsystem extends SubsystemBase {
  private ObsidianCANSparkMax m_leaderMotor;
  private ObsidianCANSparkMax m_followerMotor;

  private DutyCycleEncoder m_encoder;

  private ObsidianPID m_obsidianPID;
  private Servo m_servo;

  private double m_setPoint;



  public ClimbSubsystem(int CanID1,int CanID2, int encoderDIO, int ServoID){
    m_leaderMotor = new ObsidianCANSparkMax(CanID1,MotorType.kBrushless,true,80);
    m_followerMotor = new ObsidianCANSparkMax(CanID2,MotorType.kBrushless,true,80);
    m_servo = new Servo(ServoID);
    
    /*
    SparkMaxConfig followconfig = new SparkMaxConfig();
    followconfig
      .follow(CanID1)
      .inverted(true)
      .limitSwitch.reverseLimitSwitchEnabled(false).forwardLimitSwitchEnabled(false);


    m_followerMotor.configure(followconfig, null, PersistMode.kPersistParameters); */

    m_encoder=new DutyCycleEncoder(encoderDIO);

    m_obsidianPID=new ObsidianPID(ClimbConstants.climbP,ClimbConstants.climbI,ClimbConstants.climbD,
                                  ClimbConstants.climbMaxAndMin,-ClimbConstants.climbMaxAndMin);
    m_obsidianPID.setInputGain(ClimbConstants.climbInputGain);



  }

  public void engageServo(){
      m_servo.setAngle(100);
  }

  public void disengageServo(){
      m_servo.setAngle(80);
  }

  public double getAbsoluteEncoderPos(){
    return m_encoder.get();
  }

  public void togglePID(double setPoint){
    m_obsidianPID.togglePID();
    m_setPoint=setPoint;
  }

  public void resetPID(){
    m_obsidianPID.resetToggle();
  }

  public boolean getPIDToggle(){
    return m_obsidianPID.getToggle();
  }

  public boolean checkCANConnections(){
    boolean returnValue=true;
    double var=0;

    try{
      var=m_leaderMotor.getDeviceId();
      var=m_followerMotor.getDeviceId();
      var=m_encoder.get();
    }catch(Exception e){
      returnValue=false;
    }

    return returnValue;
  }

  public void runWithLimits(double speed){
    if(speed<0){
      if(getAbsoluteEncoderPos()<ClimbConstants.climbLimitHigh){
        run(speed);
      }else{
        System.out.println("AT LIMIT HIGH ROTATE");
        stop();
      }
  }else{
       if(getAbsoluteEncoderPos()>ClimbConstants.climbLimitLow){
        run(speed);
      }else{
        System.out.println("AT LIMIT LOW ROTATE");
        stop();
      }
    }
  }

  public void run(double speed){
    m_leaderMotor.set(speed);
    m_followerMotor.set(speed);
    
  }

  public void stop(){
    m_leaderMotor.stopMotor();
    m_followerMotor.stopMotor();
  }

  @Override
  public void periodic() {
    if(m_obsidianPID.getToggle()){
      double pidOutput=m_obsidianPID.calculatePercentOutput(getAbsoluteEncoderPos(),m_setPoint);
      System.out.println(pidOutput);
      m_leaderMotor.set(pidOutput);
      m_followerMotor.set(pidOutput);
    }
  }
}
