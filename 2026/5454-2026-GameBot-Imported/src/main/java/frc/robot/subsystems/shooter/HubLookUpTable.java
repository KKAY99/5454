package frc.robot.subsystems.shooter;

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

import java.util.TreeMap;

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
        // Distance (m), Shooter Speed (RPS), Trajectory Angle (°), Time of Flight (s)
        // KrakenX60 shooting 226g ball - optimized for constant RPS ~75
        // Trajectory angles: 90° = straight up, 45° = maximum distance
        addEntry(77.46,  63, 0.00, 1.00);  // Close shot - nearly straight up
        addEntry(70.29,  61, 0.00, 1.00);  // Close shot - nearly straight up
        addEntry(64,  60, 0.00, 1.00);  // Close shot - nearly straight up
        addEntry(57,  59, 0.00, 1.00);  // Close shot - nearly straight up
        addEntry(24,  55, 0.00, 1.00);  // Close shot - nearly straight up
        addEntry(89,  67, 0.00, 1.00);  // Close shot - nearly straight up
        addEntry(97,  68, 0.11, 1.00);  // Close shot - nearly straight up
        addEntry(117,  67, 0.11, 1.00);  // Close shot - nearly straight up
        addEntry(129,  70, 0.11, 1.00);  // Close shot - nearly straight up
        addEntry(146,  70.5, 0.11, 1.00);  // Close shot - nearly straight up
        addEntry(166,  73, 0.14, 1.00);  // Close shot - nearly straight up
        addEntry(178,  73, 0.20, 1.00);  // Close shot - nearly straight up
        addEntry(204,  78, 0.24, 1.00);  // Close shot - nearly straight up
        addEntry(223,  79, 0.24, 1.00);  // Close shot - nearly straight up
        // Max distance 
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