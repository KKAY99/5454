package frc.robot.utils;
import au.grapplerobotics.LaserCan;
import frc.robot.Constants.LaserCanConstants;

public class Lasercan{
    public LaserCan m_laserCan;

    public Lasercan(int id){
        m_laserCan=new LaserCan(id);
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