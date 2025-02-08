package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkBase.ControlType;
import org.littletonrobotics.junction.Logger;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import frc.robot.Constants.ElevatorConstants;
import frc.robot.utilities.ObsidianCANSparkMax;
import frc.robot.utilities.ObsidianCanandcolor;

public class ElevatorSubsystem extends SubsystemBase {
  private ObsidianCANSparkMax m_motor1;
  private SparkClosedLoopController m_loopController;
  private ObsidianCanandcolor m_canAndColor;

  private double m_speed=0;
  private double m_positionTarget=0;
  private String m_controlType;

  public ElevatorSubsystem(int CanID_1,int canColorID) {
    m_motor1 = new ObsidianCANSparkMax(CanID_1, MotorType.kBrushless,true,40,
              ElevatorConstants.elevatorPK1,ElevatorConstants.elevatorIK1,ElevatorConstants.elevatorDK1,ElevatorConstants.elevatorMaxAndMinK1,
              ElevatorConstants.elevatorPK2,ElevatorConstants.elevatorIK2,ElevatorConstants.elevatorDK2,ElevatorConstants.elevatorMaxAndMinK2);
    //m_canAndColor=new ObsidianCanandcolor(canColorID);
    m_loopController=m_motor1.getClosedLoopController();

    m_controlType="Default-Velocity";
  }

  public void resetRelative(){
    m_motor1.getEncoder().setPosition(0);
  }

  public double getRelativePos(){
    return m_motor1.getEncoder().getPosition();
  }

  public void set_referance(double pos){
    m_controlType="Position";
    m_positionTarget=pos;

    m_loopController.setReference(pos,ControlType.kPosition);
  }

  public void set_referance(double pos,ClosedLoopSlot closedLoopSlot){
    m_controlType="Position";
    m_positionTarget=pos;

    m_loopController.setReference(pos,ControlType.kPosition,closedLoopSlot);
  }

  public void reset_referance(){
    m_controlType="Velocity";
    m_loopController.setReference(0, ControlType.kVelocity);
  }

  public void runWithLimits(double speed){
    if(speed<0){
      if(getRelativePos()>ElevatorConstants.elevatorHighLimit){
        motor_run(speed);
      }else{
        System.out.println("AT HIGH LIMIT ELEVATOR");
        motor_stop();
      }
    }else{
      if(getRelativePos()<ElevatorConstants.elevatorLowLimit){
        motor_run(speed);
      }else{
        System.out.println("AT LOW LIMIT ELEVATOR");
        motor_stop();
      }
    }
  }
  
  public void motor_run(double speed){
    m_motor1.set(speed);
  }

  public void motor_stop(){
    m_motor1.stopMotor();
  }

  public boolean checkCANConnections(){
    boolean returnValue=true;
    try{
      m_motor1.getDeviceId();
    }catch(Exception e){
      returnValue=false;
    }

    return returnValue;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    Logger.recordOutput("Elevator/Speed",m_speed);
    Logger.recordOutput("Elevator/ControlType", m_controlType);
    Logger.recordOutput("Elevator/PositionTarget",m_positionTarget);
    Logger.recordOutput("Elevator/Current",m_motor1.getOutputCurrent());
  }
}
