package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.GroundIntakeConstants;
import frc.robot.commands.GroundIntakeAutomatedCommand;
import frc.robot.utilities.ObsidianCANSparkMax;
import frc.robot.utilities.ObsidianPID;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkBase.ControlType;

import org.littletonrobotics.junction.Logger;

public class GroundIntakeSubsystem extends SubsystemBase {
  private ObsidianCANSparkMax m_rotateMotor;
  private ObsidianCANSparkMax m_intakeMotor;
  private DutyCycleEncoder m_encoder;
  private ObsidianPID m_obsidianPID;

  private double m_rotateSpeed=0;
  private double m_intakeSpeed=0;

  private double m_setPoint=0;
  private double m_pidOutput=0;
 
  public GroundIntakeSubsystem(int canRotateID,int canIntakeID, int encoderDIO){
    m_rotateMotor = new ObsidianCANSparkMax(canRotateID,MotorType.kBrushless,true,Constants.k40Amp);
    m_intakeMotor = new ObsidianCANSparkMax(canIntakeID,MotorType.kBrushless,true,Constants.k40Amp);
   
    m_obsidianPID=new ObsidianPID(GroundIntakeConstants.rotatePK0,GroundIntakeConstants.rotateIK0,
    GroundIntakeConstants.rotateDK0,GroundIntakeConstants.rotateMaxAndMinK0,-GroundIntakeConstants.rotateMaxAndMinK0);
    m_obsidianPID.setInputGain(GroundIntakeConstants.rotateInputGain);

    m_encoder=new DutyCycleEncoder(encoderDIO);
  }


  public double getAbsoluteEncoderPos(){
    return m_encoder.get();
  }

  public void togglePID(double setPoint,int slot){
    switch(slot){
      case 0:
        m_obsidianPID.setAllValues(GroundIntakeConstants.rotatePK0,GroundIntakeConstants.rotateIK0,GroundIntakeConstants.rotateDK0,
                                  GroundIntakeConstants.rotateMaxAndMinK0,-GroundIntakeConstants.rotateMaxAndMinK0,GroundIntakeConstants.rotateInputGain);
      break;
      case 1:
      m_obsidianPID.setAllValues(GroundIntakeConstants.rotatePK1,GroundIntakeConstants.rotateIK1,GroundIntakeConstants.rotateDK1,
                                  GroundIntakeConstants.rotateMaxAndMinK1,-GroundIntakeConstants.rotateMaxAndMinK1,GroundIntakeConstants.rotateInputGain);
      break;
      default:

    }

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
  public double getPos(){
    return m_encoder.get();
  }
  @Override
  public void periodic(){
    Logger.recordOutput("GroundIntake/IntakeSpped",m_intakeSpeed);
    Logger.recordOutput("GroundIntake/RotateSpeed",m_rotateSpeed);
    Logger.recordOutput("GroundIntake/RotateEncoder",m_encoder.get());

    if(m_obsidianPID.getToggle()){
      m_pidOutput=m_obsidianPID.calculatePercentOutput(getAbsoluteEncoderPos(),m_setPoint);
      m_rotateMotor.set(m_pidOutput);
    }
  }
}
