package frc.robot.subsystems;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase{

    private Spark m_intakeOne;
    private Spark m_intakeTwo;
    private DutyCycleEncoder m_highTower;
    private DutyCycleEncoder m_lowTower;
    public IntakeSubsystem(int motorOne, int motorTwo,int lowTower, int highTower){
        m_intakeOne= new Spark(motorOne);
        m_intakeTwo= new Spark(motorTwo);
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
            

    
}
