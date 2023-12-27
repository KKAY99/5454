package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxAlternateEncoder;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax.IdleMode;
import frc.robot.Constants;

public class ClimbSubsystem extends SubsystemBase {
    private CANSparkMax m_ClimbMotor;
    private DigitalInput m_ClimbBottomLimitSwitch;
    private DigitalInput m_ClimbTopLimitSwitch;
    private RelativeEncoder m_ClimbEncoder;
    // private Counter m_ClimbLimitSwitchCounter;
    /** Creates a new ExampleSubsystem. */
    public ClimbSubsystem(Integer ClimbPort, Integer bottomLimitSwitchPort, Integer topLimitSwitchPort) {
        m_ClimbMotor = new CANSparkMax(ClimbPort, MotorType.kBrushed);
        
        m_ClimbEncoder=m_ClimbMotor.getEncoder(SparkMaxRelativeEncoder.Type.kQuadrature, 2048);
        m_ClimbEncoder.setPosition(0);
        m_ClimbEncoder.setInverted(true);
        m_ClimbMotor.setInverted(true); 
        m_ClimbMotor.setIdleMode(IdleMode.kBrake);

        m_ClimbMotor.setOpenLoopRampRate(0.25);
        m_ClimbBottomLimitSwitch = new DigitalInput(bottomLimitSwitchPort);
        // m_ClimbLimitSwitchCounter=new Counter(m_ClimLimitSwitch);
        m_ClimbTopLimitSwitch = new DigitalInput(topLimitSwitchPort);

    }

    public boolean hitBottomLimit() {
      //  return (m_ClimbBottomLimitSwitch.get());
        //if(m_ClimbBottomLimitSwitch.get() ||(m_ClimbEncoder.getPosition() <= Constants.climbDownLimit)){
            System.out.println("Hit Bottom Limit - " + m_ClimbBottomLimitSwitch.get() + " --" + m_ClimbEncoder.getPosition());
            if(m_ClimbBottomLimitSwitch.get()){
          
            System.out.println("Hit Bottom Limit - " + m_ClimbBottomLimitSwitch.get() + " --" + m_ClimbEncoder.getPosition());
            return true;
        }else
        {
            return false;
        }
    
    } 

    public boolean hitTopLimit() {
        return false;
      /*
        //  return (m_ClimbTopLimitSwitch.get());
      if(m_ClimbEncoder.getPosition() >= Constants.climbUpLimit){
        System.out.println("Top Bottom Limit - " + m_ClimbEncoder.getPosition());
        return true;
      }else
      {
          return false;
      }
      */
    }

    public void forceBottom() {
        while(m_ClimbBottomLimitSwitch.get() == false) {
           m_ClimbMotor.set(Constants.climbDownSpeed / 4);
        }

        stop();
        m_ClimbEncoder.setPosition(0);
    }

    public void run(double speed) {
        System.out.println("setting climb speed=" + speed);
        m_ClimbMotor.set(speed);
    }

    public boolean stopForLimit(double speed) {
        if (((hitBottomLimit() == false) || (speed > 0)) && (hitTopLimit() == false || (speed < 0))) {
            return false;
        } else {
            return true;
        }
    }
    
    public double getEncoderPosition(){
        return m_ClimbEncoder.getPosition();
 //    return 0;
    }

    public void stop() {
        System.out.println("setting climb speed to 0 - STOP function");

        m_ClimbMotor.set(0.0);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }
}
