package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxAlternateEncoder;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.common.control.PidController;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;

import com.revrobotics.CANSparkMax.IdleMode;
import frc.robot.Constants;
import com.revrobotics.RelativeEncoder;


public class RotateArmSubsystem extends SubsystemBase {
    private CANSparkMax m_RotateMotorLeader;
    private CANSparkMax m_RotateMotor;
    RelativeEncoder m_rotateLeaderEncoder;
    RelativeEncoder m_rotateFollowerEncoder;
    private boolean m_homed=false;
    private double m_homeAngle;
    private double m_frontLimit;
    private double m_backLimit;
    private DutyCycleEncoder m_AbsoluteEncoder;
    private double kp=0.6;
    private double ki=0.01;
    private double kd=0.0;
   private PIDController m_rotorPID=new PIDController(kp,ki,kd);

    public RotateArmSubsystem(int motorPort1,int motorPort2,int absoluteEncoderPort,double homeAngle,double frontLimit,double backLimit){
        m_RotateMotorLeader = new CANSparkMax(motorPort1 , MotorType.kBrushless);   
        m_RotateMotorLeader.setIdleMode(CANSparkMax.IdleMode.kBrake);
        m_RotateMotorLeader.setSmartCurrentLimit(30);
        m_rotateLeaderEncoder = m_RotateMotorLeader.getEncoder();
        m_AbsoluteEncoder= new DutyCycleEncoder(absoluteEncoderPort);

        m_RotateMotor = new CANSparkMax(motorPort2 , MotorType.kBrushless);  
        m_RotateMotor.setSmartCurrentLimit(30); 
        m_RotateMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        m_rotateFollowerEncoder = m_RotateMotor.getEncoder();
        m_RotateMotor.follow(m_RotateMotorLeader, true);
        
        m_homeAngle=homeAngle;
        m_frontLimit=frontLimit;
        m_backLimit=backLimit;
      }
    

    @Override
    public void periodic() {
      // This method will be called once per scheduler run
    //System.out.println("ABS Encoder" + m_AbsoluteEncoder.get()  + " Rel Encoder " + m_rotateEncoder.getPosition());
    }

    public void rotate(double power){
      //System.out.println("Setting Power on Rotate - " + power);
      if(checkForwardSoftLimit(power) || checkBackwardSoftLimit(power)){
        power=0;
      }
        m_RotateMotorLeader.set(power);
    }
    public void doPIDTest(){
      PIDTest(0);
      PIDTest(0.4);
      PIDTest(0.5);
      PIDTest(0.6);
    }
 
    private void PIDTest(double setpoint)
    { 
      System.out.println("Current Pos :" + getAbsolutePos() + " SetPoint:" + setpoint + "  PID Calculated Output:" + 
                     m_rotorPID.calculate(getAbsolutePos(),setpoint));
    
    }
 
 
    public boolean checkBackwardSoftLimit(double speed)
    {
     // System.out.println("Hit Backward Limit");
     return (getAbsolutePos()>=m_backLimit && (speed<0));
    }

    public boolean checkForwardSoftLimit(double speed)
    {
    //  System.out.println("Hit Forward Limit");
     return (getAbsolutePos()<=m_frontLimit && (speed>0));
    }

    public boolean hasHitBackSoftLimit(){
      return getAbsolutePos()>=m_homeAngle;
    }

    public boolean hasHitForwardSoftLimit(){
      return getAbsolutePos()<=m_homeAngle;
    }

    public void stopRotate(){
      m_RotateMotorLeader.set(0);
    }
    public double getAbsolutePos(){
      return m_AbsoluteEncoder.get();
    }

    public double getRelativeRotatePos(){
      return m_rotateLeaderEncoder.getPosition();
    }
    public void SetZero(){
      m_rotateLeaderEncoder.setPosition(0);
      m_rotateFollowerEncoder.setPosition(0);
    }
    public boolean hitHomeAngle(){
      //TODO: IMPLEMENT
      return (getAbsolutePos()==m_homeAngle);
     
    }

    public void setHomed(boolean value){
      m_homed=value;
    }
  
    public boolean hasHomed(){
      return m_homed;
    }

    public void disableRotateBrakeMode(){
      m_RotateMotorLeader.setIdleMode(IdleMode.kCoast);
      m_RotateMotor.setIdleMode(IdleMode.kCoast);
    }

    public void resetRotateBrakeModeToNormal(){
      m_RotateMotorLeader.setIdleMode(IdleMode.kBrake);
      m_RotateMotor.setIdleMode(IdleMode.kBrake);
    }
  
    @Override
    public void simulationPeriodic() {
      // This method will be called once per scheduler run during simulation
    }
}
