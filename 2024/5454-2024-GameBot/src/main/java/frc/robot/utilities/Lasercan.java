package frc.robot.utilities;
import au.grapplerobotics.LaserCan;
import frc.robot.Constants.LaserCanConstants;

public class Lasercan{
    private Lasercan m_highTower;
    private Lasercan m_lowTower;

    public Lasercan(int lowTurretid,int highTurretid){
        m_highTower=new LaserCan(id);
    }

    public double GetDistanceInMM(){
        System.out.println(m_laserCan.getMeasurement().status);

        return m_laserCan.getMeasurement().distance_mm;
    }

    public boolean BreakBeam(){
        boolean returnValue=false;
        if(GetDistanceInMM()<LaserCanConstants.distanceToReflector+LaserCanConstants.deadBand){
            returnValue=true;
        }

        return returnValue;
    }
}