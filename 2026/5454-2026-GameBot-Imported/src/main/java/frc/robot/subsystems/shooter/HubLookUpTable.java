package frc.robot.subsystems.shooter;

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

import java.util.TreeMap;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** 
 * Lookup table for hub shooting parameters based on distance.
 * Supports linear interpolation between data points.
 */
public class HubLookUpTable {
    
    /** Data structure to hold shooting parameters */
    public static class ShootingParameters {
        public final double shooterSpeed;     // RPS (Revolutions Per Second)
        public final double hoodPosition;  // Degrees
        public final double timeOfFlight;     // Seconds
        
        public ShootingParameters(double shooterSpeed, double hoodPosition, double timeOfFlight) {
            this.shooterSpeed = shooterSpeed;
            this.hoodPosition = hoodPosition;
            this.timeOfFlight = timeOfFlight;
        }
    }
    
    // TreeMap automatically sorts by distance (key)
    private final TreeMap<Double, ShootingParameters> lookupTable;
    
    public HubLookUpTable() {
        lookupTable = new TreeMap<>();
        initializeLookupTable();
    }
    
    /** Initialize the lookup table with known data points */
    private void initializeLookupTable() {
        // Distance (m), Shooter Speed (RPS), Hood Angle (°), Time of Flight (s)
        // KrakenX60 shooting 226g ball - optimized for constant RPS ~75
        
        addEntry(2,  50, 0.0, 1.00); 
        addEntry(2.3,  53, 0.0, 1.00);  
        addEntry(2.42,  53, 0.1, 1.00);  
        addEntry(2.63,  54.54, 0.1, 1.00);  
        addEntry(2.78,  54.54, 0.17, 1.00);  
        addEntry(2.98,  54.54, 0.20, 1.00);  
        addEntry(3.36,  54.54, 0.30, 1.00);  
        addEntry(3.51,  56, 0.30, 1.00);  
        addEntry(3.68,  56.5, 0.32, 1.00);  
       
       addEntry(3.68,  56.5, 0.32, 1.00);  
       addEntry(3.91,  58, 0.32, 1.00);  
       addEntry(4.19,  58.5, 0.32, 1.00);  
       addEntry(4.49,  59.5, 0.35, 1.00);  
       addEntry(4.78,  62, 0.35, 1.00);  
       addEntry(5.61,  65, 0.36, 1.00);  
       //GIVE IT ALL THE POWER
       addEntry(9,  66, 0.36, 1.00);  
       
  
    }
    
    /** Add an entry to the lookup table */
    public void addEntry(double distance, double shooterSpeed, double hoodPosition, double timeOfFlight) {
        lookupTable.put(distance, new ShootingParameters(shooterSpeed, hoodPosition, timeOfFlight));
    }
    
    /** 
     * Get interpolated shooting parameters for a given distance 
     * @param distance Distance to target in meters
     * @return Interpolated shooting parameters
     */
    public ShootingParameters getParameters(double distance) {
        // Check if exact match exists
        if (lookupTable.containsKey(distance)) {
            return lookupTable.get(distance);
        }
        
        // Get the surrounding values
        Double lowerKey = lookupTable.floorKey(distance);
        Double upperKey = lookupTable.ceilingKey(distance);
        
        // Handle edge cases
        if (lowerKey == null) {
            return lookupTable.get(upperKey); // Below minimum distance
        }
        if (upperKey == null) {
            return lookupTable.get(lowerKey); // Above maximum distance
        }
        
        // Perform linear interpolation
        ShootingParameters lower = lookupTable.get(lowerKey);
        ShootingParameters upper = lookupTable.get(upperKey);
        
        double ratio = (distance - lowerKey) / (upperKey - lowerKey);
        
        double interpolatedSpeed = lerp(lower.shooterSpeed, upper.shooterSpeed, ratio);
        double interpolatedAngle = lerp(lower.hoodPosition, upper.hoodPosition, ratio);
        double interpolatedTime = lerp(lower.timeOfFlight, upper.timeOfFlight, ratio);
        
        //Speed Multiplier
        double multiplier = SmartDashboard.getNumber("Speed Adjuster:",1);
        if(!(multiplier==1)){
            System.out.println("Changing Speed due to Multiplier-" + multiplier);
                //increase by multiplier percent
                interpolatedSpeed=interpolatedSpeed*(multiplier);
            

        }
        return new ShootingParameters(interpolatedSpeed, interpolatedAngle, interpolatedTime);
    }

    /** Linear interpolation helper */
    private double lerp(double start, double end, double ratio) {
        return start + (end - start) * ratio;
    }
    
    /** Get shooter speed for a given distance */
    public double getShooterSpeed(double distance) {
        return getParameters(distance).shooterSpeed;
    }
    
    /** Get trajectory angle for a given distance */
    public double getTrajectoryAngle(double distance) {
        return getParameters(distance).hoodPosition;
    }
    
    /** Get time of flight for a given distance */
    public double getTimeOfFlight(double distance) {
        return getParameters(distance).timeOfFlight;
    }
}