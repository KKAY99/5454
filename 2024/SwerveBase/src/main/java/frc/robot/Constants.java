 
package frc.robot;

import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

public final class Constants {
    
        
    public static final class InputControllers {
        public static final int kXboxDrive = 0;
        public static final int kXboxOperator = 1;
        public static final int kCustomController = 2;
    }
    /** General robot constants  from 3512*/
    public static final class GeneralConstants {
        // Enable or disable competition mode
        public static final boolean tuningMode = true;

        // Joystick axis deadband for the swerve drive
        public static final double swerveDeadband = 0.1;

        public static final double voltageComp = 10.0;

        // Hold time on motor brakes when disabled
        public static final double wheelLockTime = 10;

        public static final double robotMass = (148 - 20.3) * 0.453592;
        public static final double chassisMass = robotMass;
        public static final Translation3d chassisCG = new Translation3d(0, 0, Units.inchesToMeters(8));
        public static final double loopTime = 0.13;
    }
   public static final class Swerve {
    public static final Translation2d flModuleOffset = new Translation2d(0.4, 0.4);
    public static final Translation2d frModuleOffset = new Translation2d(0.4, -0.4);
    public static final Translation2d blModuleOffset = new Translation2d(-0.4, 0.4);
    public static final Translation2d brModuleOffset = new Translation2d(-0.4, -0.4);

    public static final double maxModuleSpeed = 4.5; // M/S

    public static final HolonomicPathFollowerConfig pathFollowerConfig = new HolonomicPathFollowerConfig(
      new PIDConstants(5.0, 0, 0), // Translation constants 
      new PIDConstants(5.0, 0, 0), // Rotation constants 
      maxModuleSpeed, 
      flModuleOffset.getNorm(), // Drive base radius (distance from center to furthest module) 
      new ReplanningConfig()
    );
  }
  
}