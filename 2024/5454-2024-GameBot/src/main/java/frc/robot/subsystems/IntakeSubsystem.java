package frc.robot.subsystems;
import com.fasterxml.jackson.annotation.JsonTypeInfo.Id;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import org.littletonrobotics.junction.Logger;
import frc.robot.Constants;
import frc.robot.utilities.Lasercan;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase{
    private IntakeSubsystemIO m_intakeIO;
    private IntakeSubsystemIOInputsAutoLogged m_intakeAutoLogged=new IntakeSubsystemIOInputsAutoLogged();

    private CANSparkMax m_intakeOne;
    private CANSparkMax m_intakeTwo;

    private double m_currentSpeed;

    private boolean m_intakeToggle=false;

    public IntakeSubsystem(int motorOne,int motorTwo,IntakeSubsystemIO intakeIO){
        m_intakeOne= new CANSparkMax(motorOne,MotorType.kBrushless);
        m_intakeOne.setSmartCurrentLimit(Constants.k30Amp);
        m_intakeOne.setIdleMode(IdleMode.kBrake);
        m_intakeTwo= new CANSparkMax(motorTwo,MotorType.kBrushless);
        m_intakeTwo.setIdleMode(IdleMode.kBrake);
        m_intakeOne.setSmartCurrentLimit(Constants.k30Amp);

        m_intakeIO=intakeIO;
    }

    public void runIntake(double speed){
        m_currentSpeed=speed;
        m_intakeOne.set(speed);
        m_intakeTwo.set(speed);
    }

    public void stopIntake(){
        m_currentSpeed=0;
        m_intakeOne.set(0);
        m_intakeTwo.set(0);
    }
            
    public void setBrakeOn(){
        m_intakeOne.setIdleMode(IdleMode.kBrake);
        m_intakeTwo.setIdleMode(IdleMode.kBrake);
    
    } 

    public void setCoastOn(){
        m_intakeOne.setIdleMode(IdleMode.kCoast);
        m_intakeTwo.setIdleMode(IdleMode.kCoast);
    }

    public void ToggleIntake(double speed){
        if(m_intakeToggle){
            m_intakeToggle=false;
            stopIntake();
        }else{
            m_intakeToggle=true;
            runIntake(speed);
        }
    }

    @Override
    public void periodic(){
        m_intakeIO.updateInputs(m_intakeAutoLogged);

        //Logger.processInputs("IntakeSubsystem",m_intakeAutoLogged);
        Logger.recordOutput("Intake/IntakeSpeed",m_currentSpeed);
    }
}
