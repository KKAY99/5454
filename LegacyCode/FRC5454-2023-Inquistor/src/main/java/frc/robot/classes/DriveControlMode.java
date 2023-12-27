package frc.robot.classes;

public class DriveControlMode {
    private boolean m_FieldOrientated=true;
    public DriveControlMode(){

    }
    public void setFieldMode(){
        m_FieldOrientated=true;
    }
    public void setRobotMode(){
        m_FieldOrientated=false;
    }
    public boolean isFieldOrientated(){
        return m_FieldOrientated;
    }
}
