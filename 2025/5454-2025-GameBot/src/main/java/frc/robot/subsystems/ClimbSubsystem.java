package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ClimbConstants;
import frc.robot.Constants.DunkinDonutConstants;
import frc.robot.utilities.ObsidianCANSparkMax;
import frc.robot.utilities.ObsidianPID;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.HardwareLimitSwitchConfigs;
import com.revrobotics.servohub.ServoHub.ResetMode;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkFlexExternalEncoder;
import edu.wpi.first.wpilibj.Servo;

public class ClimbSubsystem extends SubsystemBase {
  private ObsidianCANSparkMax m_climbMotor;

  private DutyCycleEncoder m_encoder;

  private ObsidianPID m_obsidianPID;
  private Servo m_servo;

  private double m_setPoint;
  private double m_pidOutput;

  public ClimbSubsystem(int CanID1,int CanID2, int encoderDIO, int ServoID){
    m_climbMotor = new ObsidianCANSparkMax(CanID1,MotorType.kBrushless,true,Constants.k80Amp);
    m_servo = new Servo(ServoID);

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
      var=m_climbMotor.getDeviceId();
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
        //System.out.println("AT LIMIT HIGH ROTATE");
        stop();
      }
  }else{
       if(getAbsoluteEncoderPos()>ClimbConstants.climbLimitLow){
        run(speed);
      }else{
        //System.out.println("AT LIMIT LOW ROTATE");
        stop();
      }
    }
  }

  public void run(double speed){
    m_climbMotor.set(speed); 
  }

  public void stop(){
    m_climbMotor.stopMotor();
  }

  @Override
  public void periodic(){
    if(m_obsidianPID.getToggle()){
      m_pidOutput=m_obsidianPID.calculatePercentOutput(getAbsoluteEncoderPos(),m_setPoint);
      m_climbMotor.set(m_pidOutput);
    }

    Logger.recordOutput("Climb/Setpoint",m_setPoint);
    Logger.recordOutput("Climb/PidOutput",m_pidOutput);
  }
}
