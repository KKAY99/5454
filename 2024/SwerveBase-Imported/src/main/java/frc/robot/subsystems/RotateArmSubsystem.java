package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxAlternateEncoder;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import frc.robot.Constants;
import com.revrobotics.RelativeEncoder;


public class RotateArmSubsystem extends SubsystemBase {
    private CANSparkMax m_RotateMotor;

    public RotateArmSubsystem(int motorPort1){
        m_RotateMotor = new CANSparkMax(motorPort1, MotorType.kBrushless);  
        m_RotateMotor.setSmartCurrentLimit(30); 
        m_RotateMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
      }
    

    @Override
    public void periodic() {
      // This method will be called once per scheduler run
    //System.out.println("ABS Encoder" + m_AbsoluteEncoder.get()  + " Rel Encoder " + m_rotateEncoder.getPosition());
    }

    public void rotate(double power){
        m_RotateMotor.set(power);
    }
 
    

    public void stopRotate(){
      m_RotateMotor.set(0);
    }
    
}
