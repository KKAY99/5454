package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.DigitalInput;

import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase{
    private TalonSRX m_leftShootMotor;
    private TalonSRX m_rightShootMotor;
    private TalonSRX m_SnowMotors;

    private double m_shootMotorSpeed;
    private double m_snowMotorSpeed;
    private double m_maxSnowVoltage;
    private DigitalInput m_IntakeLimitSwitch;

    private static final double kIntakeStallVoltage=13;
    public ShooterSubsystem(int leftShootMotorPort, int rightShootMotorPort, int snowMotorPort,double snowMotorSpeed,int limitSwitch){
        m_leftShootMotor = new TalonSRX(leftShootMotorPort);
        m_rightShootMotor = new TalonSRX(rightShootMotorPort);
        m_leftShootMotor.setNeutralMode(NeutralMode.Brake);
        m_rightShootMotor.setNeutralMode(NeutralMode.Brake);
        m_SnowMotors =  new TalonSRX(snowMotorPort);
        m_IntakeLimitSwitch = new DigitalInput(limitSwitch);
        m_snowMotorSpeed=snowMotorSpeed;
    }

    public void intakeCube(double shootMotorSpeed){
        m_leftShootMotor.set(ControlMode.PercentOutput,shootMotorSpeed);
        m_rightShootMotor.set(ControlMode.PercentOutput,-shootMotorSpeed);
        m_SnowMotors.set(ControlMode.PercentOutput, -m_snowMotorSpeed);
        
        double currentVoltage=Math.abs(m_SnowMotors.getMotorOutputVoltage());
        if(currentVoltage>m_maxSnowVoltage){
            m_maxSnowVoltage=currentVoltage;
        }
    }

    public void spinUpShooterMotors(double shootMotorSpeed){
        m_leftShootMotor.set(ControlMode.PercentOutput,shootMotorSpeed);
        m_rightShootMotor.set(ControlMode.PercentOutput,shootMotorSpeed);
    }

    public void expellCube(double shootMotorSpeed){
        m_leftShootMotor.set(ControlMode.PercentOutput,-shootMotorSpeed/2);
        m_rightShootMotor.set(ControlMode.PercentOutput,shootMotorSpeed);
        m_SnowMotors.set(ControlMode.PercentOutput, m_snowMotorSpeed);
        
    }
   
    public double getMaxSnowVoltage(){
        return m_maxSnowVoltage;
    }
    public boolean hasCube(){
      /*   System.out.print("Max Voltage - " + m_maxSnowVoltage + "Current Voltage - " + m_SnowMotors.getBusVoltage());
        if(m_maxSnowVoltage>kIntakeStallVoltage){
            return true;
        }else {
            return false;
        }
      */
      if(m_IntakeLimitSwitch.get()){
        return false;
      }else{
        return true;
      }
      }
    public void stop(){
        System.out.println("Stopping");
        m_leftShootMotor.set(ControlMode.PercentOutput,0);
        m_rightShootMotor.set(ControlMode.PercentOutput,0);
        m_SnowMotors.set(ControlMode.PercentOutput, 0);
        m_maxSnowVoltage=0; // reset max voltage
    } 
    
   
    
    
}
