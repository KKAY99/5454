package frc.robot.subsystems;

import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import frc.robot.Constants.DunkinDonutConstants;
import frc.robot.utilities.ObsidianCANSparkMax;
import com.ctre.phoenix6.hardware.CANcoder;


import org.littletonrobotics.junction.Logger;

import com.revrobotics.spark.*;

public class DunkinDonutSubsystem extends SubsystemBase {
  private CANcoder m_CANcoder;
  private ObsidianCANSparkMax m_coralMotor;
  private ObsidianCANSparkMax m_algaeMotor;
  private ObsidianCANSparkMax m_rotateMotor;
  private double m_rotateSpeed=0;
  private double m_coralSpeed=0;
  private double m_algaeSpeed=0;
  private SparkClosedLoopController m_loopController;

  private boolean m_algaeToggle=false;
  
  public DunkinDonutSubsystem(int coralCanID, int algaeCanID, int rotateCanID,int canCoderID) {
    m_coralMotor = new ObsidianCANSparkMax(coralCanID, MotorType.kBrushless, true);
    m_algaeMotor= new ObsidianCANSparkMax(algaeCanID, MotorType.kBrushless, true);
    m_rotateMotor = new ObsidianCANSparkMax(rotateCanID, MotorType.kBrushless, true,40,
                    DunkinDonutConstants.dunkinP,DunkinDonutConstants.dunkinI,DunkinDonutConstants.dunkinD,DunkinDonutConstants.dunkinMaxAndMin);
    m_CANcoder = new CANcoder(canCoderID);
  
    m_loopController=m_rotateMotor.getClosedLoopController();
  }

  public double getAbsoluteEncoderPos(){
    return m_CANcoder.getAbsolutePosition().getValueAsDouble();
  }

  public void runRotateWithLimits(double speed){
    if(speed<0){
      if(getAbsoluteEncoderPos()<DunkinDonutConstants.relativeHighLimitABS){
        run_rotatemotor(speed);
      }else{
        System.out.println("AT LIMIT HIGH ROTATE");
        stop_rotatemotor();
      }
    }else{
      if(getAbsoluteEncoderPos()>DunkinDonutConstants.relativeLowLimitABS){
        run_rotatemotor(speed);
      }else{
        System.out.println("AT LIMIT LOW ROTATE");
        stop_rotatemotor();
      }
    }
  }

  public void resetRotateRelative(){
    m_rotateMotor.getEncoder().setPosition(0);
  }

  public double get_rotatemotorpos(){
    return m_rotateMotor.getEncoder().getPosition();
  }

  public void set_referance(double pos){
    m_loopController.setReference(pos, ControlType.kPosition);
  }

  public void reset_referance(){
    m_loopController.setReference(0, ControlType.kVelocity);
  }

  public void run_rotatemotor(double speed){
    m_rotateSpeed=speed;
    m_rotateMotor.set(speed);
  }

  public void stop_rotatemotor(){
    m_rotateSpeed=0;
    m_rotateMotor.stopMotor();
  }
  
  public void runCoralMotor(double speed){
    m_coralSpeed=speed;
    m_coralMotor.set(speed); 
  }
  public void runAlgaeMotor(double speed){
    m_algaeSpeed=speed;
    m_algaeMotor.set(speed);
  }
  public void stopCoralMotor(){
    m_coralMotor.stopMotor();
    m_coralSpeed=0;
  }

  public void stopAlgeaMotor(){
    m_algaeMotor.stopMotor();
    m_algaeSpeed=0;
  }

  public void algeaToggle(double speed){
    if(!m_algaeToggle){
      runAlgaeMotor(speed);
      m_algaeToggle=true;
    }else{
      stopAlgeaMotor();
      m_algaeToggle=false;
    }
  }

  public boolean checkCANConnections(){
    boolean returnValue=true;
    try{
      m_CANcoder.getDeviceID();
      m_algaeMotor.getDeviceId();
      m_coralMotor.getDeviceId();
      m_rotateMotor.getDeviceId();
    }catch(Exception e){
      returnValue=false;
    }

    return returnValue;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  // This method will be called once per scheduler run
    Logger.recordOutput("Dunkin/RotateSpeed", m_rotateSpeed);
    Logger.recordOutput("Dunkin/CoralSpeed", m_coralSpeed);
    Logger.recordOutput("Dunkin/AlgeaSpeed",m_algaeSpeed);
    //Logger.recordOutput("Dunkinr/CurrentPosition",get_motor1pos());    
  }
}
