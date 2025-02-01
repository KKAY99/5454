package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import frc.robot.utilities.ObsidianCANSparkMax;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.spark.*;

public class DunkinDonutSubsystem extends SubsystemBase {
  private ObsidianCANSparkMax m_coralMotor;
  private ObsidianCANSparkMax m_algaeMotor;
  private ObsidianCANSparkMax m_rotateMotor;
  private double m_rotateSpeed=0;
  private double m_coralSpeed=0;
  private double m_algaeSpeed=0;
  private SparkClosedLoopController m_loopController;
  
  public DunkinDonutSubsystem(int coralCanID, int algaeCanID, int rotateCanID) {
    m_coralMotor = new ObsidianCANSparkMax(coralCanID, MotorType.kBrushless, true);
    m_algaeMotor= new ObsidianCANSparkMax(algaeCanID, MotorType.kBrushless, true);
    m_rotateMotor = new ObsidianCANSparkMax(rotateCanID, MotorType.kBrushless, true);

    m_loopController=m_rotateMotor.getClosedLoopController();
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
  public void stopCoralMotor(double speed){
    m_coralMotor.stopMotor();
    m_coralSpeed=0;
  }
  public void stopAlgeaMotor(double speed){
    m_algaeMotor.stopMotor();
    m_algaeSpeed=0;
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
