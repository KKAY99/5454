package frc.robot.utilities;
import frc.robot.Constants;

public class ShotTable {
    
    private double parseTable(double distance,int column,boolean calculateValue){
    try{    
        boolean bExceededDistance=false;
        int tableLoop=0;
        int rows=Constants.Shooter.distanceLookup.length;
        double currentValue=0;
        double lastValue=0;
        double nextDistance=0;
        double lastDistance=0;
        double currentDistance=0;
        
        currentValue=Constants.Shooter.distanceLookup[tableLoop][column];
        
        do{
            lastValue=currentValue;
            currentValue=Constants.Shooter.distanceLookup[tableLoop][column];
            lastDistance=currentDistance;
            currentDistance=Constants.Shooter.distanceLookup[tableLoop][Constants.Shooter.columnDistance];
            tableLoop++;
            if(tableLoop<rows){
                nextDistance=Constants.Shooter.distanceLookup[tableLoop][Constants.Shooter.columnDistance];
            } else{
                nextDistance=999999;
            }
            } while (nextDistance<distance); 
        //table loop is pointing to the 
        if(calculateValue){
            //TODO: Calculate value using median 
           // System.out.println(lastValue);
           // System.out.println(currentValue);
           // System.out.println(lastDistance);
           // System.out.println(currentDistance);
            double x=(distance-lastDistance) / (currentDistance - lastDistance)
            double z=(currentValue - lastValue) * x;
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
        return parseTable(distance,Constants.Shooter.columnAngle,false);
    }
    public double getVelocity(double distance){
        return parseTable(distance,Constants.Shooter.columnVelocity,true);
    }
    public double getShotTime(double distance){
        return parseTable(distance,Constants.Shooter.columnShotTime,false);
    }
}
