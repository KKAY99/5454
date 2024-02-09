package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.TurretConstants;
import frc.robot.utilities.Limelight;

import java.io.Console;

import javax.sound.midi.SysexMessage;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;

public class TurretSubsystem extends SubsystemBase{
  private TurretSubsystemIO m_turretIO;
  private TurretSubsystemIOInputsAutoLogged m_turretAutoLogged=new TurretSubsystemIOInputsAutoLogged();

  private CANSparkMax m_turretMotor;

  private SparkMaxPIDController m_pidController;

  private DigitalInput m_limitSwitch;

  private RelativeEncoder m_encoder;

  private double m_speed;

  private double kTurretP=Constants.TurretConstants.turretP;
  private double kTurretI=Constants.TurretConstants.turretI;
  private double kTurretD=Constants.TurretConstants.turretD;

  public TurretSubsystem(int turretMotorPort, int limitSwitchPort, TurretSubsystemIO turretIO){
    m_turretIO=turretIO;
    m_turretMotor=new CANSparkMax(turretMotorPort,MotorType.kBrushless);
    //m_limitSwitch=new DigitalInput(limitSwitchPort);
    m_encoder=m_turretMotor.getEncoder();
    m_pidController=m_turretMotor.getPIDController();
    m_pidController.setP(kTurretP);
    m_pidController.setI(kTurretI);
    m_pidController.setD(kTurretD);
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
    m_turretMotor.set(power);
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
  public boolean isAtPosition(double targetPos,double deadband){
    //if gap between target pos and current position is less than deadband we return true
    return Math.abs((Math.abs(GetEncoderValue())-Math.abs(targetPos)))<deadband;
  
  }

  public boolean IsAtHardLimit(){
    if(m_limitSwitch!=null){
    return m_limitSwitch.get();
    }
    else{
      return false;
    }
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

  public void TurretSetReference(double pos){
    m_pidController.setReference(pos,ControlType.kPosition);
  }

  public void ResetPIDReference(){
    m_pidController.setReference(0,ControlType.kVelocity);
  }

  public void SetEncoderToZero(){
    m_encoder.setPosition(0);
  }
  public void setBrakeOn(){
      m_turretMotor.setIdleMode(IdleMode.kBrake);  
    } 
  public void setCoastOn(){
         m_turretMotor.setIdleMode(IdleMode.kCoast);
    }
  
 public void aimAtGoal(Pose2d robotPose, Translation2d goal, boolean aimAtVision) {
  //FIX: TO DO Implement AimAtGoal 
  /* Translation2d robotToGoal = goal.minus(robotPose.getTranslation());
    double angle = Math.atan2(robotToGoal.getY(), robotToGoal.getX());
    angle = Math.PI + angle - robotPose.getRotation().getRadians();

    angle = MathUtils.toUnitCircAngle(angle);

    if (m_trackTarget) {
      Limelight.enable();
    } else {
      Limelight.disable();
    }

    SmartDashboard.putNumber("Turret Set Angle", angle);

    if (aimAtVision && Limelight.valid()) {
      angle = getMeasurement() - Limelight.tx();
    }

    m_desiredAngle = angle;

    if (angle < TurretConstants.kLow) {
      angle = TurretConstants.kLow;
    } else if (angle > TurretConstants.kHigh) {
      angle = TurretConstants.kHigh;
    }

    double neoRevs = angle / TurretConstants.kRatio / (2 * Math.PI);

    m_controller.setReference(neoRevs, ControlType.kSmartMotion);
 */
  }

  @Override
  public void periodic(){
    m_turretIO.updateInputs(m_turretAutoLogged);

    Logger.processInputs("TurretSubsystem",m_turretAutoLogged);
    Logger.recordOutput("TurretSpeed",m_speed);
    Logger.recordOutput("TurretEncoder",GetEncoderValue());
    Logger.recordOutput("TurretLimit",IsAtHardLimit());

    //SmartDashboard.putNumber("TurretEncoder",GetEncoderValue());
  }
}
