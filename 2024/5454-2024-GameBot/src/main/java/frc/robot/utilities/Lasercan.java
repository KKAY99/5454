package frc.robot.utilities;
import org.littletonrobotics.junction.Logger;
import au.grapplerobotics.LaserCan;
import au.grapplerobotics.LaserCan.RegionOfInterest;
import au.grapplerobotics.LaserCan.TimingBudget;
import frc.robot.Constants.LaserCanConstants;


public class Lasercan{
    private LasercanIO m_laserCanIO;
    private LasercanIOInputsAutoLogged m_laserCanAutoLogged=new LasercanIOInputsAutoLogged();

    private LaserCan m_highTurret;
    private LaserCan m_lowTurret;

    public Lasercan(int lowTurretid,int highTurretid){
        try{
            m_lowTurret=new LaserCan(lowTurretid);
            m_highTurret=new LaserCan(highTurretid);

            RegionOfInterest lowROI=new RegionOfInterest(6,4,6,4);
            m_lowTurret.setRegionOfInterest(lowROI);

            RegionOfInterest highROI=new RegionOfInterest(6,4,6,4);
            m_highTurret.setRegionOfInterest(highROI);

            m_lowTurret.setTimingBudget(TimingBudget.TIMING_BUDGET_33MS);
            m_highTurret.setTimingBudget(TimingBudget.TIMING_BUDGET_33MS);
        } catch (Exception e){
            System.out.print("LaserCan Init Failed: Error - " + e.getMessage());
        }
    }

    public double GetDistanceInMMLow(){
        double returnValue=0.0;

        if(m_lowTurret.getMeasurement()==null){
            returnValue=0.0;
        }else{
            returnValue=m_lowTurret.getMeasurement().distance_mm;
        }

        return returnValue;
    }

    public double GetDistanceInMMHigh(){
        double returnValue=0.0;

        if(m_lowTurret.getMeasurement()==null){
            returnValue=0.0;
        }else{
            returnValue=m_highTurret.getMeasurement().distance_mm;
        }

        return returnValue;
    }

    public boolean LowTurretBreakBeam(){
        boolean returnValue=false;

        if(m_lowTurret.getMeasurement()==null){
            returnValue=false;
        }else if(GetDistanceInMMLow()<LaserCanConstants.distanceToReflectorLow+LaserCanConstants.deadBand){
            returnValue=true;
        }

        return returnValue;
    }

    public boolean HighTurretBreakBeam(){
        boolean returnValue=false;

        if(m_lowTurret.getMeasurement()==null){
            returnValue=false;
        }else if(GetDistanceInMMHigh()<LaserCanConstants.distanceToReflectorHigh+LaserCanConstants.deadBand){
            returnValue=true;
        }

        return returnValue;
    }

    public void LaserCanPeriodic(){
        m_laserCanIO.updateInputs(m_laserCanAutoLogged);

        //Logger.processInputs("LaserCan",m_laserCanAutoLogged);
        Logger.recordOutput("LaserCan/LowTurretBreakBeam",LowTurretBreakBeam());
        Logger.recordOutput("LaserCan/HighTurretBreakBeam",HighTurretBreakBeam());
    } 
}