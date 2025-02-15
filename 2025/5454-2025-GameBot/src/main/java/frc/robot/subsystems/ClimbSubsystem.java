package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbConstants;
import frc.robot.Constants.DunkinDonutConstants;
import frc.robot.utilities.ObsidianCANSparkMax;
import frc.robot.utilities.ObsidianPID;

public class ClimbSubsystem extends SubsystemBase {
  private ObsidianCANSparkMax m_motor;
  private DutyCycleEncoder m_encoder;

  private ObsidianPID m_obsidianPID;

  private double m_setPoint;

  public ClimbSubsystem(int CanID,int encoderDIO){
    m_motor = new ObsidianCANSparkMax(CanID,MotorType.kBrushed,true,80);
    m_encoder=new DutyCycleEncoder(encoderDIO);

    m_obsidianPID=new ObsidianPID(ClimbConstants.climbP,ClimbConstants.climbI,ClimbConstants.climbD,
                                  ClimbConstants.climbMaxAndMin,-ClimbConstants.climbMaxAndMin);
    m_obsidianPID.setInputGain(ClimbConstants.climbInputGain);
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
    m_motor.set(speed);
  }

  public void stop(){
    m_motor.stopMotor();
  }

  @Override
  public void periodic() {
    if(m_obsidianPID.getToggle()){
      double pidOutput=m_obsidianPID.calculatePercentOutput(getAbsoluteEncoderPos(),m_setPoint);
      System.out.println(pidOutput);
      m_motor.set(pidOutput);
    }
  }
}
