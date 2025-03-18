package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import frc.robot.Constants;
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
  private ObsidianCANSparkMax m_rotateMotor;
  private ObsidianCANSparkMax m_coralIndexer;

  private SparkClosedLoopController m_loopController;

  private ObsidianPID m_obsidianPID;
  private RelativeEncoder m_rotateRelative;
  private RelativeEncoder m_coralRelative;

  private DigitalInput m_coralLimitSwitch;
  private DigitalInput m_indexerLimitSwitch;


  private double m_rotateSpeed=0;
  private double m_coralSpeed=0;
  private double m_algaeSpeed=0;
  private double m_setPoint=0;
  private double m_pidOutput=0;

  private boolean m_algaeToggle=false;
  private boolean m_shouldRunPID=false;
  
  public DunkinDonutSubsystem(int coralCanID,int algaeCanID1,int rotateCanID,int canCoderID, int limitSwitch, int coralIndexerID, int indexerLimitSwitchID) {
    m_coralMotor = new ObsidianCANSparkMax(coralCanID, MotorType.kBrushless, true, Constants.k80Amp,DunkinDonutConstants.coralP,DunkinDonutConstants.coralI,DunkinDonutConstants.coralD);
    m_algaeMotor1= new ObsidianCANSparkMax(algaeCanID1, MotorType.kBrushless, true);
    m_rotateMotor = new ObsidianCANSparkMax(rotateCanID, MotorType.kBrushless, true,Constants.k40Amp);
    m_coralIndexer = new ObsidianCANSparkMax(coralIndexerID, MotorType.kBrushless, true);
                    //DunkinDonutConstants.dunkinP,DunkinDonutConstants.dunkinI,DunkinDonutConstants.dunkinD,DunkinDonutConstants.dunkinMaxAndMin);
    m_CANcoder = new CANcoder(canCoderID);
    m_rotateRelative=m_rotateMotor.getEncoder();
    m_coralRelative = m_coralMotor.getEncoder();
    m_obsidianPID=new ObsidianPID(DunkinDonutConstants.clawPIDkP,DunkinDonutConstants.clawPIDkI,DunkinDonutConstants.clawPIDkD,
                                  DunkinDonutConstants.clawPIDMaxAndMin,-DunkinDonutConstants.clawPIDMaxAndMin);
    m_obsidianPID.setInputGain(DunkinDonutConstants.clawPIDInputGain);

    m_coralLimitSwitch = new DigitalInput(limitSwitch);
    m_indexerLimitSwitch = new DigitalInput(indexerLimitSwitchID);

    m_loopController = m_coralMotor.getClosedLoopController();
    
  }

  public double getAbsoluteEncoderPos(){
    return m_CANcoder.getAbsolutePosition().getValueAsDouble();
  }

  public double getCoralPos(){
    return m_coralRelative.getPosition();
  }

  public double getTargetPos(double ticks){
    return m_coralRelative.getPosition() + ticks;
  }

  public void runRotateWithLimits(double speed){
    if(speed<0){
      if(getAbsoluteEncoderPos()<DunkinDonutConstants.relativeHighLimitABS){
        run_rotatemotor(speed);
      }else{
        //System.out.println("AT LIMIT HIGH ROTATE");
        stop_rotatemotor();
      }
    }else{
      if(getAbsoluteEncoderPos()>DunkinDonutConstants.relativeLowLimitABS){
        run_rotatemotor(speed);
      }else{
        //System.out.println("AT LIMIT LOW ROTATE");
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

  public boolean isCoralAtIndexerLimit(){
    if(m_indexerLimitSwitch!=null){
      System.out.println("Indexer limit"+ m_indexerLimitSwitch.get());
      return m_indexerLimitSwitch.get();
    }else{
      return false;
    }
  }

  public boolean isCoralAtBoxLimit(){
    if(m_coralLimitSwitch!=null){
      return m_coralLimitSwitch.get()?false:true;
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
  }
  public void stopCoralMotor(){
    m_coralMotor.stopMotor();
    m_coralSpeed=0;
  }

  public void stopAlgeaMotor(){
    m_algaeMotor1.stopMotor();
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

  public void resetAlgeaToggle(){
    m_algaeToggle=false;
  }

  public void toggleLocalPid(double setPoint){
    m_setPoint=setPoint;
    m_obsidianPID.togglePID();
  }

  public boolean getToggle(){
    return m_obsidianPID.getToggle();
  }

  public void resetShouldRunPID(){
    m_obsidianPID.resetToggle();
  }

  public boolean checkCANConnections(){
    boolean returnValue=true;
    double var=0;
    
    try{
      var=m_CANcoder.getDeviceID();
      var=m_algaeMotor1.getDeviceId();
      var=m_coralMotor.getDeviceId();
      var=m_rotateMotor.getDeviceId();
    }catch(Exception e){
      returnValue=false;
    }

    return returnValue;
  }

  @Override
  public void periodic() {
    Logger.recordOutput("Dunkin/RotateSpeed", m_rotateSpeed);
    Logger.recordOutput("Dunkin/CoralSpeed", m_coralSpeed);
    Logger.recordOutput("Dunkin/CoralAmp", m_coralMotor.getOutputCurrent());
    Logger.recordOutput("Dunkin/AlgeaSpeed",m_algaeSpeed);   
    SmartDashboard.putBoolean("Dunkin/ShouldRunPID",m_shouldRunPID);
    SmartDashboard.putNumber("Dunkin/PIDOutput",m_pidOutput);
    SmartDashboard.putNumber("Dunkin/Setpoint",m_setPoint);

    if(m_obsidianPID.getToggle()){
      m_pidOutput=m_obsidianPID.calculatePercentOutput(getAbsoluteEncoderPos(),m_setPoint);
      m_rotateMotor.set(m_pidOutput);
    }
  }
}
