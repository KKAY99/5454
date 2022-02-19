package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimbSubsystem extends SubsystemBase {
    CANSparkMax m_ClimbMotor;

    /** Creates a new ExampleSubsystem. */
    public ClimbSubsystem(Integer ClimbPort) {
        m_ClimbMotor = new CANSparkMax(ClimbPort, MotorType.kBrushed);   
        m_ClimbMotor.setInverted(false);
        m_ClimbMotor.setOpenLoopRampRate(0.25);
    }

    public void run(double speed) 
    {   System.out.println("setting climb speed=" + speed);
        m_ClimbMotor.set(speed);
    }

    

    public void stop() {
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
