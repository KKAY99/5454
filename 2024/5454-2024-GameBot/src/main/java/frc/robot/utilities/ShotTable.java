package frc.robot.utilities;
import edu.wpi.first.math.InterpolatingMatrixTreeMap;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.units.Velocity;
import frc.robot.Constants;
import frc.robot.Constants.ShooterTable;
import frc.robot.Constants.ShooterConstants;
import edu.wpi.first.math.Num;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

public class ShotTable {
    private InterpolatingDoubleTreeMap shotVelocity= new InterpolatingDoubleTreeMap();
    private InterpolatingDoubleTreeMap angle= new InterpolatingDoubleTreeMap();
    private InterpolatingDoubleTreeMap shotTime= new InterpolatingDoubleTreeMap();
   
    public ShotTable(){
        loadtables();
    }

    private void loadtables (){
        //READ CONSTANTS
        double distance;
        double velocity;
        double shotTimerow;
        double anglerow;
        int dataRows=Constants.ShooterTable.distanceLookup.length;
        for(int rowLoop=0;rowLoop<dataRows;rowLoop++){
            distance=Constants.ShooterTable.distanceLookup[rowLoop][ShooterTable.columnDistance];
            velocity=Constants.ShooterTable.distanceLookup[rowLoop][ShooterTable.columnVelocity];
            anglerow=Constants.ShooterTable.distanceLookup[rowLoop][ShooterTable.columnAngle];
            shotTimerow=Constants.ShooterTable.distanceLookup[rowLoop][ShooterTable.columnShotTime];
            shotVelocity.put(distance,velocity);
            angle.put(distance,anglerow);
            shotTime.put(distance,shotTimerow);
        }
      
    }

    private double DONOTUSEparseTable(double distance,int column,boolean calculateValue){
    try{    
       
        int tableLoop=0;
        int rows=Constants.ShooterTable.distanceLookup.length;
        double currentValue=0;
        double lastValue=0;
        double nextDistance=0;
        double nextValue=0;
        double lastDistance=0;
        double currentDistance=0;
        
        currentValue=Constants.ShooterTable.distanceLookup[tableLoop][column];
        
        do{
            lastValue=currentValue;
            currentValue=Constants.ShooterTable.distanceLookup[tableLoop][column];
            lastDistance=currentDistance;
            currentDistance=Constants.ShooterTable.distanceLookup[tableLoop][Constants.ShooterTable.columnDistance];
            tableLoop++;
            if(tableLoop<rows){
                nextDistance=Constants.ShooterTable.distanceLookup[tableLoop][Constants.ShooterTable.columnDistance];
                nextValue=Constants.ShooterTable.distanceLookup[tableLoop][column];
        
            } else{
                nextDistance=999999;
            }
            } while (nextDistance<distance); 
        //table loop is pointing to the 
        if(calculateValue){
            //TODO: Calculate value using median 
            //System.out.println(currentValue);
            //System.out.println(nextValue);
            //System.out.println(currentDistance);
            //System.out.println(nextDistance);
            double x=(distance-currentDistance) / (nextDistance - currentDistance); 
            double y=(nextValue - currentValue) * x;
            double z=y+currentValue;
            System.out.println(distance + "^-^" + x + "^" + y +  "^^"+z);
            //System.out.println();
            //System.out.println();
            
            return z;
        }else {
            return currentValue;
        }
    }
    catch(Exception e){
        System.out.println("Exception: "+ e.getMessage());
        return 0;
    }
        
    }
    public double getAngle(double distance){
        return angle.get(distance);
    }
    public double getVelocity(double distance){
        return shotVelocity.get(distance);
    }
    public double getShotTime(double distance){
        return shotVelocity.get(distance);
    }
}
