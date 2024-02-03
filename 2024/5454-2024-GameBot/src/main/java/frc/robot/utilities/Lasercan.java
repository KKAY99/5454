package frc.robot.utilities;
import au.grapplerobotics.LaserCan;
import frc.robot.Constants.LaserCanConstants;

public class Lasercan{
    private LaserCan m_highTurret;
    private LaserCan m_lowTurret;

    public Lasercan(int lowTurretid,int highTurretid){
        m_lowTurret=new LaserCan(lowTurretid);
        m_highTurret=new LaserCan(highTurretid);
    }

    public double GetDistanceInMMLow(){
        System.out.println(m_lowTurret.getMeasurement().status);

        return m_lowTurret.getMeasurement().distance_mm;
    }

    public double GetDistanceInMMHigh(){
        System.out.println(m_highTurret.getMeasurement().status);

        return m_highTurret.getMeasurement().distance_mm;
    }

    public boolean LowTurretBreakBeam(){
        boolean returnValue=false;
        if(GetDistanceInMMLow()<LaserCanConstants.distanceToReflectorLow+LaserCanConstants.deadBand){
            returnValue=true;
        }

        return returnValue;
    }

    public boolean HighTurretBreakBeam(){
        boolean returnValue=false;
        if(GetDistanceInMMHigh()<LaserCanConstants.distanceToReflectorHigh+LaserCanConstants.deadBand){
            returnValue=true;
        }

        return returnValue;
    }
}