package frc.robot.subsystems;
import com.fasterxml.jackson.annotation.JsonTypeInfo.Id;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase{

    private CANSparkMax m_intakeOne;
    private CANSparkMax m_intakeTwo;
    private DutyCycleEncoder m_highTower;
    private DutyCycleEncoder m_lowTower;
    public IntakeSubsystem(int motorOne, int motorTwo,int lowTower, int highTower){
        m_intakeOne= new CANSparkMax(motorOne,MotorType.kBrushless);
        m_intakeOne.setIdleMode(IdleMode.kBrake);
        m_intakeTwo= new CANSparkMax(motorTwo,MotorType.kBrushless);
        m_intakeTwo.setIdleMode(IdleMode.kBrake);
        m_lowTower = new DutyCycleEncoder(lowTower);
        m_highTower = new DutyCycleEncoder(highTower);
        }

    public void runIntake(double speed){
        m_intakeOne.set(speed);
        m_intakeTwo.set(speed);
    }
    public void stopIntake(){
        m_intakeOne.set(0);
        m_intakeTwo.set(0);
    }
    
    public boolean isLowerTowerDetected(){
        return m_lowTower.get()==1;
    }
    public boolean isHigherTowerDetected(){
          return m_highTower.get()==1;
    }
            
    public void setBrakeOn(){
      m_intakeOne.setIdleMode(IdleMode.kBrake);
      m_intakeTwo.setIdleMode(IdleMode.kBrake);
    
    } 
    public void setCoastOn(){
         m_intakeOne.setIdleMode(IdleMode.kCoast);
         m_intakeTwo.setIdleMode(IdleMode.kCoast);
    }
    
}
