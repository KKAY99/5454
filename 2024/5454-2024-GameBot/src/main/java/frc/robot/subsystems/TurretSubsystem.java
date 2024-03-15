package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.TurretConstants;
import frc.robot.utilities.Limelight;
import frc.robot.utilities.ShotTable;

import java.io.Console;
import java.util.logging.LogManager;

import javax.sound.midi.SysexMessage;

import org.littletonrobotics.junction.LogDataReceiver;
import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;

public class TurretSubsystem extends SubsystemBase{
  private TurretSubsystemIO m_turretIO;
  private TurretSubsystemIOInputsAutoLogged m_turretAutoLogged=new TurretSubsystemIOInputsAutoLogged();
  private ShotTable m_shotTable = new ShotTable();

  private CANSparkMax m_turretMotor;

  private SparkMaxPIDController m_pidController;

  private DigitalInput m_limitSwitch;

  private RelativeEncoder m_encoder;

  private boolean m_hasSetReference;
  
  private Limelight m_limeLight;
  private double m_speed;
  private double m_targetAngle;

  private double kTurretP=Constants.TurretConstants.turretP;
  private double kTurretI=Constants.TurretConstants.turretI;
  private double kTurretD=Constants.TurretConstants.turretD;

  public TurretSubsystem(int turretMotorPort, int limitSwitchPort, Limelight limelight,TurretSubsystemIO turretIO){
     m_turretIO=turretIO;
    m_limeLight=limelight;
    //HACKm_turretMotor=new CANSparkMax(turretMotorPort,MotorType.kBrushless);
    //HACKm_turretMotor.setSmartCurrentLimit(Constants.k15Amp);
    m_limitSwitch=new DigitalInput(limitSwitchPort);
    //HACKm_encoder=m_turretMotor.getEncoder();
    //HACKm_pidController=m_turretMotor.getPIDController();
    //HACKm_pidController.setP(kTurretP);
    //HACKm_pidController.setI(kTurretI);
    //HACK//HACKm_pidController.setD(kTurretD);
    //HACKm_turretMotor.burnFlash(); //ensure all settings are saved if a a browout happens

  }

  public boolean IsOnTarget(){
    boolean returnValue=false;
    double limelightDis=m_limeLight.getDistance();
    double multiplier=m_shotTable.getDistanceMultiplier(limelightDis);
    double x=m_limeLight.getXRaw()+m_shotTable.getCrosshairOffset(limelightDis); 
    
    m_limeLight.setOffSet(m_shotTable.getCrosshairOffset(limelightDis));

    if(m_limeLight.isTargetAvailible()){
      if(Math.abs(x)<Constants.LimeLightValues.limeLightDeadBand*multiplier){
          returnValue=true;
      }
     }
     return returnValue;
    } 

  public void TrackTarget(boolean bool){
    double speed=0;
    double limelightDis=m_limeLight.getDistance();
    double x=m_limeLight.getXRaw()+m_shotTable.getCrosshairOffset(limelightDis);
    double multiplier=m_shotTable.getDistanceMultiplier(limelightDis);
    Logger.recordOutput("Turret/TrackXTarget",x);
   
   
    if(m_limeLight.isTargetAvailible()){
        if(Math.abs(x)<Constants.LimeLightValues.limeLightDeadBand*multiplier){
          speed=0;
        }else if(Math.abs(x)<Constants.LimeLightValues.closeXCheck*multiplier){
          speed=Constants.LimeLightValues.limeLightTrackSpeed2;
        }else if(Math.abs(x)<Constants.LimeLightValues.medXCheck*multiplier){
          speed=Constants.LimeLightValues.limeLightTrackSpeed3;
        }else if(Math.abs(x)<Constants.LimeLightValues.farXCheck*multiplier){
          speed=Constants.LimeLightValues.limeLightTrackSpeed4;
        }else{
          speed=Constants.LimeLightValues.limeLightTrackSpeed5;
        }
        if(x<0 && speed!=0){
          RunCheckLimits(speed);
          m_speed=speed;
        }else if(x>0 && speed!=0){
          RunCheckLimits(-speed);
          m_speed=-speed;
        }else{
          stop();          
        }
     }else {
     // search mode
        //set default speed
        if(m_speed==0){
          m_speed=speed=Constants.LimeLightValues.limeLightTrackSpeed5;
        }
        double oldspeed=m_speed; // speed gets set to zero in checklimits
        if(RunCheckLimits(m_speed)) {
          System.out.println("reverse speed" + oldspeed);
          m_speed=0-oldspeed;  // reverse speed when we hit a limit
        }
      
     }
  }

    public void TESTINGTrackTarget(boolean bool){
    double speed=0;
    double limelightDis=m_limeLight.getDistance();
    double x=m_limeLight.getXRaw()+m_shotTable.getCrosshairOffset(limelightDis);
    double multiplier=m_shotTable.getDistanceMultiplier(limelightDis);
    Logger.recordOutput("Turret/TrackXTarget",x);
   
   
    if(m_limeLight.isTargetAvailible()){
        if(Math.abs(x)<Constants.LimeLightValues.limeLightDeadBand*multiplier){
          speed=0;
        }else if(Math.abs(x)<Constants.LimeLightValues.closerXCheck*multiplier){
          speed=Constants.LimeLightValues.limeLightTrackSpeed0;
        }else if(Math.abs(x)<Constants.LimeLightValues.closeXCheck*multiplier){
          speed=Constants.LimeLightValues.limeLightTrackSpeed1TEST;
        }else if(Math.abs(x)<Constants.LimeLightValues.closeToMediumXCheck*multiplier){
          speed=Constants.LimeLightValues.limeLightTrackSpeed2TEST;
        }else if(Math.abs(x)<Constants.LimeLightValues.medXCheck*multiplier){
          speed=Constants.LimeLightValues.limeLightTrackSpeed3TEST;
        }else if(Math.abs(x)<Constants.LimeLightValues.farXCheck*multiplier){
          speed=Constants.LimeLightValues.limeLightTrackSpeed4TEST;
        }else{
          speed=Constants.LimeLightValues.limeLightTrackSpeed5TEST;
        }
        if(x<0 && speed!=0){
          RunCheckLimits(speed);
          m_speed=speed;
        }else if(x>0 && speed!=0){
          RunCheckLimits(-speed);
          m_speed=-speed;
        }else{
          stop();          
        }
     }else {
     // search mode
        //set default speed
        if(m_speed==0){
          m_speed=speed=Constants.LimeLightValues.limeLightTrackSpeed5;
        }
        double oldspeed=m_speed; // speed gets set to zero in checklimits
        if(RunCheckLimits(m_speed)) {
          System.out.println("reverse speed" + oldspeed);
          m_speed=0-oldspeed;  // reverse speed when we hit a limit
        }
      
     }
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
   //HACK m_turretMotor.set(power);
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
      System.out.println("stopping right limit");
      stop();
      returnValue=true;
    }else if(IsAtLeftLimit()&&IsRotatingToLeft()){
      System.out.println("stopping left limit");
      stop();
      returnValue=true;
    }else{
      RunTurretMotor(power);
    }
    return returnValue;
  }

  public void stop(){
    m_speed=0;
  //HACK  m_turretMotor.stopMotor();
  }
  public boolean isAtPosition(double targetPos,double deadband){
    
    //if gap between target pos and current position is less than deadband we return true
    //HACK return Math.abs((Math.abs(GetEncoderValue())-Math.abs(targetPos)))<deadband;
    return true;
  }

  public boolean IsAtHardLimit(){
    if(m_limitSwitch!=null){
    return m_limitSwitch.get();
    }else{
      return false;
    }
  }

  public boolean IsAtLeftLimit(){
    return GetEncoderValue()>=Constants.TurretConstants.softLimitLeftHigh;
  }

  public boolean IsAtRightLimit(){
    return GetEncoderValue()<=Constants.TurretConstants.softLimitRightLow;
  }

  public boolean IsRotatingToLeft(){
    return m_speed>0;
  }

  public boolean IsRotatingToRight(){
    return m_speed<0;
  }

  public double GetEncoderValue(){
    //HACKreturn m_encoder.getPosition();
    return 0;
  }

  public void TurretSetReference(double pos){
    m_hasSetReference=true;
    m_targetAngle=pos;
    //HACKm_pidController.setReference(pos,ControlType.kPosition);
  }

  public void ResetPIDReference(){
    m_hasSetReference=false;
    //HACKm_pidController.setReference(0,ControlType.kVelocity);
  }

  public void SetEncoder(double pos ){
    //HACKm_encoder.setPosition(pos);
  }
  
  public void setBrakeOn(){
    //HACKm_turretMotor.setIdleMode(IdleMode.kBrake);  
  } 
  
    public void setCoastOn(){
    //HACKm_turretMotor.setIdleMode(IdleMode.kCoast);
  }

  public double getCurrentSpeed(){
    return m_speed;
  }
  
 public void aimAtGoal(Pose2d robotPose, Translation2d goal, boolean aimAtVision) {
  //FIX: TO DO Implemetnt AimAtGoal 
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
   
    //Logger.processInputs("TurretSubsystem",m_turretAutoLogged);
    Logger.recordOutput("Turret/TurretSpeed",m_speed);
   // Logger.recordOutput("Turret/TurretEncoder",GetEncoderValue());
   // Logger.recordOutput("Turret/TurretHardLimit",IsAtHardLimit());
   // Logger.recordOutput("Turret/TurretLeftLimit",IsAtLeftLimit());
   // Logger.recordOutput("Turret/TurretRightLimit",IsAtRightLimit());
    Logger.recordOutput("Turret/TurretHasSetReference",m_hasSetReference);
    Logger.recordOutput("Turret/TurretReferenceAngle",m_targetAngle);

   // SmartDashboard.putNumber("TurretEncoder",GetEncoderValue());
  }
}
