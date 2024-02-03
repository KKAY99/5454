package frc.robot.subsystems;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Lasercan;

public class IntakeSubsystem extends SubsystemBase{
    private Spark m_intakeOne;
    private Spark m_intakeTwo;
    private Lasercan m_highTower;
    private Lasercan m_lowTower;

    public IntakeSubsystem(int motorOne, int motorTwo,int lowTower, int highTower){
        m_intakeOne= new Spark(motorOne);
        m_intakeTwo= new Spark(motorTwo);
        m_lowTower = new Lasercan(lowTower);
        m_highTower = new Lasercan(highTower);
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
        return m_lowTower.BreakBeam();
    }
    public boolean isHigherTowerDetected(){
          return m_highTower.BreakBeam();
    }
            

    
}
