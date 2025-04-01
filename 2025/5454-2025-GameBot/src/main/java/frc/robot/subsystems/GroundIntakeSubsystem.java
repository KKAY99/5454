package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utilities.ObsidianCANSparkMax;

import org.littletonrobotics.junction.Logger;

public class GroundIntakeSubsystem extends SubsystemBase {
  private ObsidianCANSparkMax m_rotateMotor;
  private ObsidianCANSparkMax m_intakeMotor;
  private DutyCycleEncoder m_encoder;
  private double m_rotateSpeed=0;
  private double m_intakeSpeed=0;
 
  public GroundIntakeSubsystem(int canRotateID,int canIntakeID, int encoderDIO){
    m_rotateMotor = new ObsidianCANSparkMax(canRotateID,MotorType.kBrushless,true,Constants.k40Amp);
    m_intakeMotor = new ObsidianCANSparkMax(canIntakeID,MotorType.kBrushless,true,Constants.k40Amp);
   
    m_encoder=new DutyCycleEncoder(encoderDIO);
  }


  public double getAbsoluteEncoderPos(){
    return m_encoder.get();
  }


  public boolean checkCANConnections(){
    boolean returnValue=true;
    double var=0;

    try{
      var=m_rotateMotor.getDeviceId();
      var=m_intakeMotor.getDeviceId();
      var=m_encoder.get();
    }catch(Exception e){
      returnValue=false;
    }

    return returnValue;
  }

  public void runWithLimits(double speed){
/*   if(speed<0){
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
    } */
  }

  public void runRotate(double speed){
    m_rotateMotor.set(speed); 
    m_rotateSpeed=speed;
  }

  public void runIntake(double speed){
    m_intakeMotor.set(speed); 
    m_intakeSpeed=speed;
  }

  public void stopRotate(){
    m_rotateMotor.stopMotor();
    m_rotateSpeed=0;
  }

  public void stopIntake(){
    m_intakeMotor.stopMotor();
    m_intakeSpeed=0;
  }

  @Override
  public void periodic(){
    Logger.recordOutput("GroundIntake/IntakeSpped",m_intakeSpeed);
    Logger.recordOutput("GroundIntake/RotateSpeed",m_rotateSpeed);
    Logger.recordOutput("GroundIntake/RotateEncoder",m_encoder.get());
  }
}
