package frc.robot.subsystems;

import java.util.function.DoubleSupplier;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.LimitSwitches;
import edu.wpi.first.wpilibj.DigitalInput;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAlternateEncoder.Type;
import com.revrobotics.CANSparkMax.IdleMode;
public class ClawSubsystem extends SubsystemBase{
    private CANSparkMax m_clawMotor;
    private double m_clawSpeed;
    private boolean m_clawIsOpen=false;
    private static boolean bClawClose=true;
    private static boolean bClawOpen=false;
  
    public ClawSubsystem(int clawMotorPort,double clawSpeed){ 
        m_clawMotor = new CANSparkMax(clawMotorPort, MotorType.kBrushed);
        m_clawMotor.setOpenLoopRampRate(0.25);
        m_clawMotor.setSmartCurrentLimit(30);  // likely gets ignored due to brushed motor
        m_clawMotor.setSecondaryCurrentLimit(40); //Set as well at 30
        m_clawMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        m_clawSpeed=clawSpeed;        
    }
    public boolean getClaw(){
      return m_clawIsOpen;
    }

    public void runClaw(double speed){
      m_clawMotor.set(speed);
    }

    public void openClaw() {
    System.out.println("Claw Open" );
      m_clawMotor.set(m_clawSpeed);
      m_clawIsOpen=false;
    }
    public void closeClaw (){
      System.out.println("Claw Close" );
      m_clawMotor.set(-m_clawSpeed);
      m_clawIsOpen=true;   
    }
    
    public void stopClaw(){
      m_clawMotor.set(0);
    }
 
    public void setClaw(boolean status){
      System.out.println("Status "+ status);
      if(status==bClawOpen){
        openClaw();
      }else {
        closeClaw();
      }

    }   
    @Override
    public void periodic() {
        // This method will be called once per scheduler run
//     System.out.println(" * Pos  " + getRotatePos() + " " + getLimitSwitch());
    }
}
