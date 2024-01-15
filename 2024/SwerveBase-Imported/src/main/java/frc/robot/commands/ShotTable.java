package frc.robot.commands;

import frc.robot.Constants;

public class ShotTable {
    
    private double parseTable(double distance, int column,boolean calculateValue){
        boolean bExceededDistance=false;
        int tableLoop=0;
        int rows=Constants.Shooter.distanceLookup.length;
        double currentValue=0;
        double lastValue=0;
        double nextDistance=0;
        double lastDistance=0;
        double currentDistance=0;
        
        lastValue=Constants.Shooter.distanceLookup[tableLoop][column];
        
        do{
            lastValue=currentValue;
            currentValue=Constants.Shooter.distanceLookup[tableLoop][column];
            lastDistance=currentDistance;
            currentDistance=Constants.Shooter.distanceLookup[tableLoop][Constants.Shooter.columnDistance];
            tableLoop++;
            nextDistance=Constants.Shooter.distanceLookup[tableLoop][Constants.Shooter.columnDistance];
        } while (nextDistance<distance && tableLoop<rows); 
        //table loop is pointing to the 
        if(calculateValue){
            //TODO: Calculate value using median 
            return currentValue;
        }else {
            return currentValue;
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
