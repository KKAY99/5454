package frc.robot.classes;

public class Indexer {
    // private Intake m_Intake;
    private Conveyor m_Conveyor;
    private DistanceSensor m_DistanceSensor;
    // private int m_StaticDistance = 120;
    private int m_HasBallDistance = 50;

    public Indexer(Intake intake, Conveyor conveyor, DistanceSensor distanceSensor) {
        // m_Intake = intake;
        m_Conveyor = conveyor;
        m_DistanceSensor = distanceSensor;
    }

    public void index() {
        // when ball is there run conveyer
        // 
        if(m_DistanceSensor.getDist(DistanceSensor.millimeters) >= m_HasBallDistance) {
            m_Conveyor.loadConveyor();
        }
        else {
            m_Conveyor.conveyorKill();
        }
    }

}