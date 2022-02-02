package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ConveyorSubsystem extends SubsystemBase {
    VictorSPX m_ConveyorMotor;

    /** Creates a new ExampleSubsystem. */
    public ConveyorSubsystem(Integer ConveryorPort) {
        //1/31 Switched to Victor
        // m_ConveyorMotor = new CANSparkMax(ConveryorPort, MotorType.kBrushless);
        m_ConveyorMotor= new VictorSPX(ConveryorPort);
        m_ConveyorMotor.setInverted(false);
        //m_ConveyorMotor.setOpenLoopRampRate(0.25);
    }

    public void run(double speed) {
        m_ConveyorMotor.set(VictorSPXControlMode.PercentOutput,speed);
    }

    

    public void stop() {
        m_ConveyorMotor.set(VictorSPXControlMode.PercentOutput,0.0);
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
