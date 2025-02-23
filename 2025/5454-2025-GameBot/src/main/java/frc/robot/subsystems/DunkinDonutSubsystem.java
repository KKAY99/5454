package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import frc.robot.Constants.DunkinDonutConstants;
import frc.robot.utilities.ObsidianCANSparkMax;
import frc.robot.utilities.ObsidianPID;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix6.hardware.CANcoder;
import org.littletonrobotics.junction.Logger;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.*;

public class DunkinDonutSubsystem extends SubsystemBase {
  private CANcoder m_CANcoder;
  private ObsidianCANSparkMax m_coralMotor;
  private ObsidianCANSparkMax m_algaeMotor1;
  private ObsidianCANSparkMax m_algaeMotor2;
  private ObsidianCANSparkMax m_rotateMotor;
  private ObsidianCANSparkMax m_coralIndexer;

  private SparkClosedLoopController m_loopController;

  
  private PIDController m_codeBoundPID;
  private ObsidianPID m_obsidianPID;
  private RelativeEncoder m_rotateRelative;
  private RelativeEncoder m_coralRelative;

  private DigitalInput m_limitSwitch;

  private double m_rotateSpeed=0;
  private double m_coralSpeed=0;
  private double m_algaeSpeed=0;

  private double m_pidInputGain=60;
  private double m_setPoint=0;
  private double m_pidOutput=0;

  private boolean m_algaeToggle=false;
  private boolean m_shouldRunPID=false;
  
  public DunkinDonutSubsystem(int coralCanID,int algaeCanID1,int algaeCanID2,int rotateCanID,int canCoderID, int limitSwitch, int coralIndexerID) {
    m_coralMotor = new ObsidianCANSparkMax(coralCanID, MotorType.kBrushless, true, 80, 0.1,0,0);
    m_algaeMotor1= new ObsidianCANSparkMax(algaeCanID1, MotorType.kBrushless, true);
    m_algaeMotor2= new ObsidianCANSparkMax(algaeCanID2, MotorType.kBrushless, true);
    m_rotateMotor = new ObsidianCANSparkMax(rotateCanID, MotorType.kBrushless, true,40);
    m_coralIndexer = new ObsidianCANSparkMax(coralIndexerID, MotorType.kBrushless, true);
                    //DunkinDonutConstants.dunkinP,DunkinDonutConstants.dunkinI,DunkinDonutConstants.dunkinD,DunkinDonutConstants.dunkinMaxAndMin);
    m_CANcoder = new CANcoder(canCoderID);
    m_rotateRelative=m_rotateMotor.getEncoder();
    m_coralRelative = m_coralMotor.getEncoder();
    m_obsidianPID=new ObsidianPID(DunkinDonutConstants.localPIDkP,DunkinDonutConstants.localPIDkI,DunkinDonutConstants.localPIDkD,
                                  DunkinDonutConstants.localPIDMaxAndMin,-DunkinDonutConstants.localPIDMaxAndMin);
    m_codeBoundPID=new PIDController(DunkinDonutConstants.localPIDkP,DunkinDonutConstants.localPIDkI,DunkinDonutConstants.localPIDkD);

    m_limitSwitch = new DigitalInput(limitSwitch);

    m_loopController = m_coralMotor.getClosedLoopController();
    
  }

  public double getAbsoluteEncoderPos(){
    return m_CANcoder.getAbsolutePosition().getValueAsDouble();
  }

  public void runCoralWithEncoder(double ticks, double speed){
    double currentPos = m_coralRelative.getPosition();
    double targetpos = currentPos + ticks;
    boolean returnvalue = false;

    while (!returnvalue){
      if(m_coralRelative.getPosition()<targetpos){
        System.out.println("less than");
        m_coralMotor.set(speed);
      }else if(m_coralRelative.getPosition()>targetpos){
        System.out.println("greater than or = to");
        m_coralMotor.stopMotor();
        returnvalue = true;
      }
    }

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

  public void runIndexer(double speed){
    m_coralIndexer.set(speed);
  }

  public void stopIndexer(){
    m_coralIndexer.stopMotor();
  }

  public boolean isCoralAtLimit(){
    if(m_limitSwitch!=null){
      return m_limitSwitch.get();
    }else{
      return false;
    }
  }

  public void resetRotateRelative(){
    m_rotateRelative.setPosition(0);
  }

  public double get_rotatemotorpos(){
    return m_rotateRelative.getPosition();
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
    m_algaeMotor1.set(speed);
    m_algaeMotor2.set(-speed);
  }
  public void stopCoralMotor(){
    m_coralMotor.stopMotor();
    m_coralSpeed=0;
  }

  public void stopAlgeaMotor(){
    m_algaeMotor1.stopMotor();
    m_algaeMotor2.stopMotor();
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

  public void toggleLocalPid(double setPoint){
    m_setPoint=setPoint;
    m_shouldRunPID=!m_shouldRunPID?true:false;
  }

  public void runLocalPID(){
    if(m_shouldRunPID){
      double calculatedSpeed=m_codeBoundPID.calculate(getAbsoluteEncoderPos()*m_pidInputGain,m_setPoint*m_pidInputGain)*-1;

      if(calculatedSpeed>DunkinDonutConstants.localPIDMaxAndMin){
        calculatedSpeed=DunkinDonutConstants.localPIDMaxAndMin;
      }else if(calculatedSpeed<-DunkinDonutConstants.localPIDMaxAndMin){
        calculatedSpeed=-DunkinDonutConstants.localPIDMaxAndMin;
      }

      m_pidOutput=calculatedSpeed;
      m_rotateMotor.set(calculatedSpeed);
    }
  }

  public boolean getShouldRunPID(){
    return m_shouldRunPID;
  }

  public void resetShouldRunPID(){
    m_shouldRunPID=false;
  }

  public boolean checkCANConnections(){
    boolean returnValue=true;
    try{
      m_CANcoder.getDeviceID();
      m_algaeMotor1.getDeviceId();
      m_algaeMotor2.getDeviceId();
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
    SmartDashboard.putBoolean("ShouldRunPID",m_shouldRunPID);
    SmartDashboard.putNumber("PIDOutput",m_pidOutput);
    SmartDashboard.putNumber("Setpoint",m_setPoint);

    runLocalPID();
  }
}
