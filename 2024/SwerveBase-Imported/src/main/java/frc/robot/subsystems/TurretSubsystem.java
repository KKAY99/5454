package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.TurretConstants;

import java.io.Console;

import javax.sound.midi.SysexMessage;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.RelativeEncoder;

public class TurretSubsystem extends SubsystemBase{
  private CANSparkMax m_turretMotor;

  private DigitalInput m_limitSwitch;

  private RelativeEncoder m_encoder;

  private double m_speed;

  public TurretSubsystem(int turretMotorPort, int limitSwitchPort){
    m_turretMotor=new CANSparkMax(turretMotorPort,MotorType.kBrushless);
    m_limitSwitch=new DigitalInput(limitSwitchPort);
    m_encoder=m_turretMotor.getEncoder();
  }

  public void TrackTarget(boolean bool){

  }
  
  public void RunTurretMotor(double power){
    //Check if motor speed is greater than limit
    if(Math.abs(power)>Constants.TurretConstants.maxTurretSpeed){
      System.out.println("Over Speed Limit: "+power);
      if(power<0){
        power=-TurretConstants.maxTurretSpeed;
      }else{
        power=TurretConstants.maxTurretSpeed;
      }
    }
    m_speed=power;
    m_turretMotor.set(m_speed);
  }

  
  public boolean RunCheckLimits(double power){
    m_speed=power;
    boolean returnValue=false;
    //return true if limit hit
    /*System.out.println("Right Limit:" + IsAtRightLimit());
    System.out.println("Left Limit:" + IsAtLeftLimit());
    System.out.println("IsRotatingtoRight: " + IsRotatingToRight());
    System.out.println("IsRotatingtoLeft: " + IsRotatingToLeft());*/

    
    if(IsAtRightLimit()&&IsRotatingToRight()){
      stop();
      returnValue=true;
    }else if(IsAtLeftLimit()&&IsRotatingToLeft()){
      stop();
      returnValue=true;
    }else{
      RunTurretMotor(power);
    }
    return returnValue;
  }

  public void stop(){
    m_speed=0;
    m_turretMotor.stopMotor();
  }

  public boolean IsAtHardLimit(){
    return m_limitSwitch.get();
  }

  public boolean IsAtLeftLimit(){
    return GetEncoderValue()<=Constants.TurretConstants.softLimitLeftLow;
  }

  public boolean IsAtRightLimit(){
    return GetEncoderValue()>=Constants.TurretConstants.softLimitRightHigh;
  }

  public boolean IsRotatingToLeft(){
    return m_speed<0;
  }

  public boolean IsRotatingToRight(){
    return m_speed>0;
  }

  public double GetEncoderValue(){
    return m_encoder.getPosition();
  }

  public void SetEncoderToZero(){
    m_encoder.setPosition(0);
  }

  @Override
  public void periodic(){
    SmartDashboard.putNumber("TurretEncoder",GetEncoderValue());
  }
}
