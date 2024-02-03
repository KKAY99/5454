package frc.robot.utils;
import au.grapplerobotics.LaserCan;
import frc.robot.Constants.LaserCanConstants;

public class Lasercan{
    public LaserCan m_laserCan;

    public Lasercan(int id){
        m_laserCan=new LaserCan(id);
    }

    public double GetDistanceInMM(){
        double returnValue=0.0;
        
        if(m_laserCan.getMeasurement()==null){
            returnValue=0.0;
        }else{
            returnValue=m_laserCan.getMeasurement().distance_mm;
        }

        return returnValue;
    }

    public boolean BreakBeam(){
        boolean returnValue=false;
        if(GetDistanceInMM()<LaserCanConstants.distanceToReflector+LaserCanConstants.deadBand){
            returnValue=true;
        }

        return returnValue;
    }
}