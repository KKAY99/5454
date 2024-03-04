 
package frc.robot;

import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.utilities.AutoPose2D;

public final class Constants {
  
  public static final double kRobotLoopTime = 0.020;
  public static final int k30Amp=30;
  public static final int k20Amp=20;
  public static final int k15Amp=15;
  public static final int k40Amp=40;
  public static final int brakeButton=3;
  public static final int pdhCAN=1;
  
  public static final class DriveConstants{
      // Joystick axis deadband for the swerve drive
    public static final double swerveDeadband = 0.1;
    public static final double MinGasPedalSpeed=0.15;
  }

  public static final class AutoConstants{

        public static final String Center="Center Start";
        public static final String LeftAmp="Left Amp";
        public static final String LeftSpeaker="Left Speaker";
        public static final String RightSpeaker="RightSpeaker";

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
        public static final double TimeDelayVal0=0;
        public static final double TimeDelayVal1=.5;
        public static final double TimeDelayVal2=1;
        public static final double TimeDelayVal3=3;
        public static final double TimeDelayVal4=5;
    
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

        public static final String noPath="No Path";
        public static final String blueShortAmpNote="BlueShortAmpNote";
        public static final String blueShortCenterNote="BlueShortCenterNote";
        public static final String blueShortSourceNote="BlueShortSourceNote";
        public static final String redShortAmpNote="RedShortAmpNote";
        public static final String redShortCenterNote="RedShortCenterNote";
        public static final String redShortSourceNote="RedShortSourceNote";
        public static final String longAmpNote1="LongAmpNote1";
        public static final String longAmpNote2="LongAmpNote2";
        public static final String longCenterNote="LongCenterNote";
        public static final String longSourceNote1="LongSourceNote2";
        public static final String longSourceNote2="LongSourceNote1";

        public static final Pose2d locationBlueShortAmpNote=new Pose2d(2.89,6.99,new Rotation2d(0));
        public static final Pose2d blueShortAmpNoteIntakeWaypoint=new Pose2d(2.89,6.99,new Rotation2d(0));
        public static final AutoPose2D blueShortAmpAutoPoses=new AutoPose2D(locationBlueShortAmpNote,blueShortAmpNoteIntakeWaypoint,
                                                                            locationBlueShortAmpNote,false);

        public static final Pose2d locationBlueShortCenterNote=new Pose2d(2.89,5.54,new Rotation2d(0));
        public static final Pose2d blueShortCenterNoteIntakeWaypoint=new Pose2d(2.89,5.54,new Rotation2d(0));
        public static final AutoPose2D blueShortCenterAutoPoses=new AutoPose2D(locationBlueShortCenterNote,blueShortCenterNoteIntakeWaypoint,
                                                                              locationBlueShortCenterNote,false);
        
        public static final Pose2d locationBlueShortSourceNote=new Pose2d(2.89,4.09,new Rotation2d(0));
        public static final Pose2d blueShortSourceNoteIntakeWaypoint=new Pose2d(2.89,4.09,new Rotation2d(0));
        public static final AutoPose2D blueShortSourceAutoPoses=new AutoPose2D(locationBlueShortSourceNote,blueShortSourceNoteIntakeWaypoint,
                                                                              locationBlueShortSourceNote,false);
        
        public static final Pose2d locationRedShortAmpNote=new Pose2d(13.67,4.09,new Rotation2d(0));
        public static final Pose2d redShortAmpNoteIntakeWaypoint=new Pose2d(13.67,4.09,new Rotation2d(0));
        public static final AutoPose2D redShortAmpAutoPoses=new AutoPose2D(locationRedShortAmpNote,redShortAmpNoteIntakeWaypoint,
                                                                          locationRedShortAmpNote,false);

        public static final Pose2d locationRedShortCenterNote=new Pose2d(13.68,5.57,new Rotation2d(0));
        public static final Pose2d redShortCenterNoteIntakeWaypoint=new Pose2d(13.68,5.57,new Rotation2d(0));
        public static final AutoPose2D redShortCenterAutoPoses=new AutoPose2D(locationRedShortCenterNote,redShortCenterNoteIntakeWaypoint,
                                                                              locationRedShortCenterNote,false);
        
        public static final Pose2d locationRedShortSourceNote=new Pose2d(13.68,7.01,new Rotation2d(0));
        public static final Pose2d redShortSourceNoteIntakeWaypoint=new Pose2d(13.68,7.01,new Rotation2d(0));
        public static final AutoPose2D redShortSourceAutoPoses=new AutoPose2D(locationRedShortSourceNote,redShortSourceNoteIntakeWaypoint,
                                                                              locationRedShortSourceNote,false);

        public static final Pose2d locationLongAmpNote=new Pose2d(8.24,  0.78,new Rotation2d(0));
        public static final Pose2d longAmpNoteRedIntakeWaypoint=new Pose2d(8.24,  0.78,new Rotation2d(0));
        public static final Pose2d longAmpNoteBlueIntakeWaypoint=new Pose2d(8.24,  0.78,new Rotation2d(0));
        public static final AutoPose2D longAmpAutoPoses=new AutoPose2D(locationLongAmpNote,false);

        public static final Pose2d locationLongAmp2Note=new Pose2d(8.24,  2.45,new Rotation2d(0));
        public static final Pose2d longAmp2NoteRedIntakeWaypoint=new Pose2d(8.24,  2.45,new Rotation2d(0));
        public static final Pose2d longAmp2NoteBlueIntakeWaypoint=new Pose2d(8.24,  2.45,new Rotation2d(0));
        public static final AutoPose2D longAmp2AutoPoses=new AutoPose2D(locationLongAmp2Note,false);

        public static final Pose2d locationLongCenterNote=new Pose2d(8.24,  4.09,new Rotation2d(0));
        public static final Pose2d longCenterNoteRedIntakeWaypoint=new Pose2d(8.24,  4.09,new Rotation2d(0));
        public static final Pose2d longCenterNoteBlueIntakeWaypoint=new Pose2d(8.24,  4.09,new Rotation2d(0));
        public static final AutoPose2D longCenterAutoPoses=new AutoPose2D(locationLongCenterNote,false);

        public static final Pose2d locationLongSource2Note=new Pose2d(8.24,  5.76,new Rotation2d(0));
        public static final Pose2d longSource2NoteRedIntakeWaypoint=new Pose2d(8.24,  5.76,new Rotation2d(0));
        public static final Pose2d longSource2NoteBlueIntakeWaypoint=new Pose2d(8.24,  5.76,new Rotation2d(0));
        public static final AutoPose2D longSource2AutoPoses=new AutoPose2D(locationLongSource2Note,false);

        public static final Pose2d locationLongSourceNote=new Pose2d(8.24,  7.44,new Rotation2d(0));
        public static final Pose2d longSourceNoteRedIntakeWaypoint=new Pose2d(8.24,  7.44,new Rotation2d(0));
        public static final Pose2d longSourceNoteBlueIntakeWaypoint=new Pose2d(8.24,  7.44,new Rotation2d(0));
        public static final AutoPose2D longSourceAutoPoses=new AutoPose2D(locationLongSourceNote,false);
       
        public static final Pose2d locationRedLongAmpWing=new Pose2d(0,0,new Rotation2d(0));
        public static final Pose2d locationBlueLongAmpWing=new Pose2d(5.28,1.91,new Rotation2d(0));
        public static final Pose2d locationRedLongSourceWing=new Pose2d(0,0,new Rotation2d(0));
        public static final Pose2d locationBlueLongSourceWing=new Pose2d(5.28,1.91,new Rotation2d(0));
       
        public static final Pose2d redCenterStartPos=new Pose2d(15.17,5.54,new Rotation2d(0));
        public static final Pose2d blueCenterStartPos=new Pose2d(1.13,5.54,new Rotation2d(0));

        public static final Pose2d redLeftAmpStartPos=new Pose2d(15.33,6.98,new Rotation2d(0));
        public static final Pose2d blueLeftAmpStartPos=new Pose2d(1.22,6.98,new Rotation2d(0));

        public static final Pose2d redLeftSpeakerStartPos=new Pose2d(15.73,4.23,new Rotation2d(0));
        public static final Pose2d blueLeftSpeakerStartPos=new Pose2d(0.73,6.72,new Rotation2d(0));

        public static final Pose2d redRightSpeakerStartPos=new Pose2d(15.73,6.72,new Rotation2d(0));
        public static final Pose2d blueRightSpeakerStartPos=new Pose2d(0.73,4.23,new Rotation2d(0));

        public static final Pose2d redMoveOutOfBoundPos=new Pose2d(13.74,4.95,new Rotation2d(0));
        public static final Pose2d blueMoveOutOfBoundPos=new Pose2d(2.18,4.90,new Rotation2d(0));
      }
        
    public static final class InputControllers {
      public static final int kXboxDrive = 0;
      public static final int kXboxOperator = 1;
      public static final int kCustomController = 2;
      public static final double kRumbleoff=0;
      public static final double kRumbleLight =0.25;
      public static final double kRumbleMedium=0.5;
      public static final double kRumbleFull=1.0;
    }

    public static final class IntakeConstants{
      public static final double intakeSpeed=1;
      public static final double intakeConveyGetNoteSpeed=1;
      //public static final double intakeConveyHasNoteSpeed=1;
      public static final double intakeConveyHasNoteSpeed=0.2;
      public static final int intakeBreakBeamport=9;
      public static final int intakeMotorPort1=18;
      public static final int intakeMotorPort2=19;

      public static final double timeToRunIntake=0.5;
      public static final double autoIntakeSpeed=1;
    }

    public static final class ConveyerConstants{
      public static final double conveySpeed=1;

      public static final int motor1Port=16;
    }

    public static final class climbConstants{
      public static final int climbPort=14;
      public static final double climbSpeed=.4;
    }

    public static final class TurretConstants{
      public static final int turretMotorPort=20;
      public static final int turretLimitSwitchPort=0;
      public static final int turretMoveTimeOut=3;
      public static final double homingPosition=0;  //encoder offset from hard limit
      public static final double deadband=1;
      public static final double maxTurretSpeed=1;
      public static final double turretSpeed=1;
      public static final double hometurretSpeed=0.10;
      public static final double softLimitRightLow=-40;
      public static final double softLimitLeftHigh=36.00;

      public static final double turretStraightPos=-37;
      public static final double turret90Pos=0;

      public static final double turretP=0.050;
      public static final double turretI=0;
      public static final double turretD=0;

      public static enum States{
        TURRET,INTAKE
      }
    }

    public static final class LaserCanConstants{
    public static final double distanceToReflectorLow=100;
    public static final double distanceToReflectorHigh=100;
    public static final double deadBand=7;

    public static final int intakeLowTowerLaserCan=2;
    public static final int intakeHighTowerLaserCan=6;
    }

   public static final class ADABreakBeamConstants{
    public static final int dioPortLow=2;
    public static final int dioPortHigh=3;
   }

   public static final class LEDConstants{
    public static final int blinkInPWM=0;
    public static final int ledPWM=9;
    public static final int ledCount=200;

    public static final double timeToWait=3;

    public static final double blinkinGreen=0.71;
    public static final double blinkinYellow=0.69;
    public static final double blinkinRed=0.61;

    public static enum PrimaryLEDStates{
      AUTO,TELEOP,INTAKELOW,INTAKEHASNOTE,TARGETLOCK,OFF
    }

    public static enum SecondaryLEDStates{
      TARGETVISIBLE,NOTARGET,TARGETLOCK,OFF
    }

    public static enum LEDColors{
      GREEN,RED,ORANGE,BLUE,PURPLE
    }

    public static enum LEDDisplayStates{
      FLASHING,SOLID,CHASING
    }
   }

    public static final class LimeLightValues {
      public static final double steeringP = 0.035;
      public static final double steeringI = 0;
      public static final double steeringD = 0.0055;
      public static final double steeringFeedForward = 0.0;

      public static final double targetHeight = 33.75; // 249 cm
      public static final double targetXPosShoot = -1.5;
      public static final double targetXPosSafeZone = 5;
      public static final double targetXPosRange=50;
      public static final double limelightTurretHeight = 17.5; //37
      public static final double limelightTurretAngle = 22.5; //40
      public static final double limelightStaticHeight = 0;
      public static final double limelightStaticAngle = 0;
      public static final double kVisionDistanceTolerance = 5;
      public static final double kVisionXTolerance = .04;
      public static final double kVisionXOffset=4;
      public static final double kVisionXMaxDistanceOffset=4.31; // was 1.7
      public static final double kVisionXMinDistanceOffset=0.91; // was 1.7

      public static final double closerXCheck=1.5;
      public static final double closeXCheck=3;
      public static final double medXCheck=9;
      public static final double farXCheck=14;

      public static final double limeLightDeadBand=0.5;

      public static final double limeLightTrackSpeed0=0.03;
      public static final double limeLightTrackSpeed1=0.05;
      public static final double limeLightTrackSpeed2=0.09;
      public static final double limeLightTrackSpeed3=0.15;
      public static final double limeLightTrackSpeed4=0.30;

      public static final String turretLimelightName="";
      public static final String staticLimelightName="static";

      public static final double podiumShotDis=0;
      public static final double midShotDis=0;
      public static final double shortShotDis=0;
      public static final double limelightDisDeadband=0.45;

      public static final int redSpeakerPipeline=0;
      public static final int blueSpeakerPipeline=1;
  }

    public static final class ShooterConstants{
      public static final double kAngleDeadband=0.15;
      public static final double kIntakeFeedAngleDeadband=10;
      public static final double kRampUpTime=1;
      public static final double kAccelCompFactor = 0.100; // in units of seconds
      public static final double autoShooterSpeed=1;
      public static final double testShooterSpeed1=-80;
      public static final double shooterIntakeSpeed=10;
      public static final double feederSpeed=-1;
      public static final double manualfeederSpeed=-0.3;
      public static final double baseSpeedDeadband=5;
      //public static final double baseMotorSpeed=-20; //rps
      public static final double baseMotorSpeed=0; //rps
      public static final double timeToRunShooter=1;
      public static final double rotateSpeed=0.99;
      public static final double rotateSlowSpeed=0.3;

      public static final double homePos=177.8;
      public static final double homeDeadband=2;
      public static final double homeRotateSpeed=0.1;
      public static final double rotateLowSoftLimit=-99;
      public static final double rotateHighSoftLimit=72; 

      public static final double podiumShotEncoderVal=-66;
      public static final double midShotEncoderVal=0;
      public static final double shortShotEncoderVal=-39;
      public static final double shooterPosDeadband=0.15;

      public static final int shooterMotorPort1=31;
      public static final int shooterMotorPort2=22;
      public static final int feederMotorPort=21;
      public static final int shooterAnglePort=23;
      public static final int encoderCanID=11;
      public static final double shooterStowAngle=-99;
      public static final double shooterVisionClearanceAngle=-65;
    }


    public static final class LimitSwitches {
      public static final int brakeButtonPort=0;
    }

    public static final class ButtonBindings{
      public static final int driverintakeToggleButtonIn=1;
      public static final int driverturret90=2;
      public static final int driverturret0=3;
      public static final int driverintakeToggleButtonOut=4;
      public static final int drivermanualShootButton=5;
      public static final int driverGyroResetButton=7;

      public static final int driverturretPOVLeft=270;
      public static final int driverturretPOVRight=90;

      public static final int operatorintakeToggleButtonIn=1;
      public static final int operatorturret90=2;
      public static final int operatorintakeConveyButtonIn=3;
      public static final int operatorintakeToggleButtonOut=4;
      public static final int operatorTurretTrack=5;
      public static final int operatorShooterIntake=6;
      public static final int operatorStow=7;
      public static final int operatorTurretToggle=8;
      public static final int operatorRotateAxis=1;
      public static final int operatorTurretAxis=0; 
      public static final int operatorClimbAxis=5;
      public static final double operatorRotateDeadband=0.2; 

      public static final int operatorturretPOVLeft=270;
      public static final int operatorturretPOVRight=90;
      public static final int operatorTurretPOVrotateUp=0;
      public static final int operatorTurretPOVrotateDown=180;

      public static final double rumbleValue=1;
      public static final double triggerDeadband=0.1;

      public static final int customShot1=0;
      public static final int customShot2=1;
      public static final int customShot3=2;
      public static final int customShot4=3;
      public static final int customShot5=4;
      public static final int customShot6=5;
    }
    public static final double customShot1Velocity1=-30;
    public static final double customShot1Velocity2=-30;
    public static final double customShot1Angle=-36;
 
    public static final double customShot2Velocity1=-30;
    public static final double customShot2Velocity2=-30;
    public static final double customShot2Angle=-30;
 
    public static final double customShot3Velocity1=-30;
    public static final double customShot3Velocity2=-30;
    public static final double customShot3Angle=-30;

    public static final double customShot4Velocity1=-30;
    public static final double customShot4Velocity2=-30;
    public static final double customShot4Angle=-30;
    
    public static final double customShot5Velocity1=-30;
    public static final double customShot5Velocity2=-30;
    public static final double customShot5Angle=-30;
    
    public static final double customShot6Velocity1=-30;
    public static final double customShot6Velocity2=-30;
    public static final double customShot6Angle=-30;
    

    /** General robot constants  from 3512*/
    public static final class GeneralConstants {
        // Enable or disable competition mode
        public static final boolean tuningMode = true;

      

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
   public static final class ShooterTable{
       public static final int columnDistance=0;
       public static final int columnVelocity1=1;
       public static final int columnVelocity2=2;
       public static final int columnAngle=3;
       public static final int columnShotTime=4;
       public static final int columnMultiplier=5;
       public static final int columnOffset=6;
      
       public static final double [][]distanceLookup={
           //distance , speed1,speed2, angle, shottime,limelight multiplier for deadband, limelight offset
          {17.5,-30,-30,-36,4,1,7.9},
          {22.45,-50,-50,-43,5,1,7.9},
          {30.5,-60,-40,-53,6,1,7.9},
          {39.4,-70,-45,-61,6,1,7.9},
          {63.08,-90,-60,-71,7,1,7.9}
      };
       public static final double [][]olddistanceLookup={
           //distance , speed, angle, shottime
          {17,0.4,-39,4},
          {22.7,0.53,-51,5},
          {31,0.53,-57.8,6},
          {37,0.65,-63.7,7},
          {39.4,0.65,-66.9,4}
      };
      

    }
    public class LEDPanel {
      public static final int UPPERPORT = 0;
      public static final int BOTTOMPORT = 9;
      public static final int FLASH_DELAY=5;

      public class Colors {
          public static final int RED = 0;
          public static final int PINK = 1;
          public static final int PURPLE = 2;
          public static final int BLUE = 3;
          public static final int CYAN = 4;
          public static final int GREEN = 5;
          public static final int YELLOW = 6;
          public static final int ORANGE = 7;
      }
  }

  public static final class SmartShooterConstants {
    public static final Translation2d kRedSpeakerLocation = new Translation2d(8.23, 4.115);
    public static final Translation2d kBlueSpeakerLocation = new Translation2d(8.23, 4.115);
 
  }
}
