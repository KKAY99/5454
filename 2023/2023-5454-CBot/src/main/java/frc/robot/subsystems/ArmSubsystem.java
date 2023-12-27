package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.PIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.Constants;

public class ArmSubsystem extends SubsystemBase {
    
    private CANSparkMax m_leftArmMotor;
    DutyCycleEncoder m_leftAbsoluteEncoder;
    private double m_homePos;
    public double m_fastSpeed;
    public double m_slowSpeed;
    public double m_rotateSpeed;
    private final double kArmBuffer=0.05;

    public ArmSubsystem(int leftArmMotorport,int encoderPort, double homePos,double fastSpeed, double slowSpeed){
    System.out.print("Motor Port " + leftArmMotorport);
    m_leftArmMotor = new CANSparkMax(leftArmMotorport , MotorType.kBrushed);   
    m_leftArmMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
    m_leftArmMotor.setSmartCurrentLimit(30);
    m_leftAbsoluteEncoder = new DutyCycleEncoder(encoderPort);
    m_homePos=homePos;
    m_fastSpeed=fastSpeed;
    m_slowSpeed=slowSpeed;

    }

    public double getEncoderPos(){
        return m_leftAbsoluteEncoder.getAbsolutePosition();
    }

    public void rotateArm(double rotateSpeed){
     setSpeed(rotateSpeed);
    }
    private boolean HitLimits(double rotateSpeed){
        return false;
        // boolean returnValue=false;
        // //HARDCODED SOFT LIMITS BAD BAD BAD BAD
        // if(rotateSpeed>0 && getEncoderPos()<.46){ // moving to back of robot
        //     returnValue=true;
        //     System.out.println("Top Soft Limit");
        // }else{
        //     if(rotateSpeed<0 && getEncoderPos()>.95){ // moving to back of robot
        //         returnValue=true;
        //         System.out.println("Bottom Soft Limit");
        //     }   
        // }
        // return returnValue;

    }
    private void setSpeed(double rotateSpeed){
    
        if(HitLimits(rotateSpeed)){
            rotateSpeed=0;
        }
        m_leftArmMotor.set(rotateSpeed);
        m_rotateSpeed=rotateSpeed;    
    }

    public void stopRotate(){
        setSpeed(0);
        }

    public void setHomePos(){
        m_homePos=m_leftArmMotor.getEncoder().getPosition();
    }
    
     public boolean goToPos(double pos){
        boolean isAtPos;
        double distancetoPos;
        distancetoPos=Math.abs(pos-getEncoderPos());

     if((distancetoPos<kArmBuffer)){
        isAtPos = true;
        stopRotate();   
     }else{
        if(pos<getEncoderPos()){
            isAtPos=false;
            setSpeed(m_slowSpeed);

        }else{
            isAtPos=false;
            setSpeed(-m_slowSpeed);
             
        }
     }
     System.out.println(isAtPos + " * * * " + distancetoPos); 
     return isAtPos;
    }
}