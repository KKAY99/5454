 
package frc.robot;

import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

public final class Constants {
    public static final class AutoConstants{
        public enum StartingLocations {
          LEFTAMP,
          LEFTSPEAKER,
          CENTER1,
          RIGHTSPEAKER
        }

        public enum AutonomousRoutines {
          REDSCORENOTE4,
          REDSCORENOTE5R,
          REDSCOREAMP,
        }

        public static final String TimeDelay0="No Delay";
        public static final String TimeDelay1=".5 seconds";
        public static final String TimeDelay2="1 seconds";
        public static final String TimeDelay3="3 seconds";
        public static final String TimeDelay4="5 seconds";
        public static final double TimeDelayVal1=0;
        public static final double TimeDelayVal2=0.5;
        public static final double TimeDelayVal3=1;
        public static final double TimeDelayVal4=3;
        public static final double TimeDelayVal5=6;

        public static final String autoMode0="0-Do Nothing";
        public static final String autoMode1="1=Score1,Move";
        public static final String autoMode2="2=Score2,Amp";
        public static final String autoMode3="3=Score2,Center";
        public static final String autoMode4="4=Score2,Source";
        public static final String autoMode5="5=Score3,Amp,Source";
        public static final String autoMode6="6=Score3,Amp,Pause,Source";
        public static final String autoMode7="7=Score3,Amp,Long Amp";
        public static final String autoMode8="8=Score3,Source,Long Source";
        public static final String autoMode9="9=Score3,Amp,Long Amp";
        public static final String autoMode10="10=Score4,Source,Long Source";
        public static final String autoMode11="11=Score4,Amp,Long Amp";

        public static final Pose2d locationRedShortAmpNote=new Pose2d(2.89,6.99,new Rotation2d(0));
        public static final Pose2d locationRedShortCenterNote=new Pose2d(2.89,5.54,new Rotation2d(0));
        public static final Pose2d locationRedShortSourceNote=new Pose2d(2.89,4.09,new Rotation2d(0));
        public static final Pose2d locationBlueShortAmpNote=new Pose2d(13.67,4.09,new Rotation2d(0));
        public static final Pose2d locationBlueShortCenterNote=new Pose2d(13.68,5.57,new Rotation2d(0));
        public static final Pose2d locationBlueShortSourceNote=new Pose2d(13.68,7.01,new Rotation2d(0));
        
        public static final Pose2d locationLongAmpNote=new Pose2d(8.24,  0.78,new Rotation2d(0));
        public static final Pose2d locationLongAmp2Note=new Pose2d(8.24,  2.45,new Rotation2d(0));
        public static final Pose2d locationLongCenterNote=new Pose2d(8.24,  4.09,new Rotation2d(0));
        public static final Pose2d locationLongSource2Note=new Pose2d(8.24,  5.76,new Rotation2d(0));
        public static final Pose2d locationLongSourceNote=new Pose2d(8.24,  7.44,new Rotation2d(0));
       
        public static final Pose2d locationRedLongRightWing=new Pose2d(5.28,  1.91,new Rotation2d(0));
       
        public static final Pose2d redCenterStartPos=new Pose2d(1.13,5.54,new Rotation2d(0));
        public static final Pose2d blueCenterStartPos=new Pose2d(0,0,new Rotation2d(0));

        public static final Pose2d redLeftAmpStartPos=new Pose2d(0,0,new Rotation2d(0));
        public static final Pose2d blueLeftAmpStartPos=new Pose2d(0,0,new Rotation2d(0));

        public static final Pose2d redLeftSpeakerStartPos=new Pose2d(0,0,new Rotation2d(0));
        public static final Pose2d blueLeftSpeakerStartPos=new Pose2d(0,0,new Rotation2d(0));

        public static final Pose2d redRightSpeakerStartPos=new Pose2d(0,0,new Rotation2d(0));
        public static final Pose2d blueRightSpeakerStartPos=new Pose2d(0,0,new Rotation2d(0));
      }
        
    public static final class InputControllers {
        public static final int kXboxDrive = 0;
        public static final int kXboxOperator = 1;
        public static final int kCustomController = 2;
    }

    public static final class RotateArm{
      public static final int armRotatePort=11;
      
      public static final double armSpeed=0.3;
    }

    public static final class LimitSwitches {
      public static final int brakeButtonPort=0;
    }

    public static final class ButtonBindings{
      public static final int rotateArmButton=1;
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
   public static final class Shooter{
       public static final int columnDistance=0;
       public static final int columnVelocity=1;
       public static final int columnAngle=2;
       public static final int columnShotTime=3;
      
       public static final double [][]distanceLookup={
           //distance , speed, angle, shottime
          {10,100,10.5,4},
          {20,200,10.5,5},
          {30,300,10.5,6},
          {40,400,10.5,7}
      };

    }
       
   
  }
  
