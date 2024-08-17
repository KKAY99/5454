package frc.robot.classes;
import java.rmi.server.ExportException;
import frc.robot.Constants.LaserCANConstants;
import au.grapplerobotics.ConfigurationFailedException;
import au.grapplerobotics.LaserCan;
import au.grapplerobotics.LaserCan.Measurement;



public class LaserCAN {
    private LaserCan laserCan;

    public LaserCAN(int canID){
        laserCan=new LaserCan(canID);
        try{
        laserCan.setRangingMode(LaserCan.RangingMode.LONG);
        laserCan.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_20MS);
        laserCan.setRegionOfInterest(new LaserCan.RegionOfInterest(8, 8, 16, 16));
        }catch(ConfigurationFailedException e){
            System.out.println("LaserCan Config Failed + " + e);
        }
    }

    public double Getdistance(){
      Measurement Distance = laserCan.getMeasurement();

      if(Distance != null && Distance.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT){
        return Distance.distance_mm;
        
       
      } else{
        return 0.0;
      }
    }

    public boolean IsBroken(){
        if(Getdistance() < LaserCANConstants.BreakPointDistance){
            return true;
        }else{
            return false;
        }
    }
}
