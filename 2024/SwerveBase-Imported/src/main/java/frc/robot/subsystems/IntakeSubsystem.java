package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
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


public class IntakeSubsystem extends SubsystemBase {
    private CANSparkMax m_intakeMotor;

    public IntakeSubsystem(int motorPort1){
        m_intakeMotor = new CANSparkMax(motorPort1,CANSparkMaxLowLevel.MotorType.kBrushless);  
        m_intakeMotor.setSmartCurrentLimit(30); 
        m_intakeMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
      }
    

    @Override
    public void periodic() {
      // This method will be called once per scheduler run
    //System.out.println("ABS Encoder" + m_AbsoluteEncoder.get()  + " Rel Encoder " + m_rotateEncoder.getPosition());
    }

    public void run(double power){
        m_intakeMotor.set(power);
    }
 
    

    public void stop(){
      m_intakeMotor.set(0);
    }
    
}
