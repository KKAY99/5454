 
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
  public static final int k26Amp=26;
  public static final int k15Amp=15;
  public static final int k40Amp=40;
  public static final int brakeButton=3;
  public static final int pdhCAN=1;
  
  public static final class DriveConstants{
      // Joystick axis deadband for the swerve drive
    public static final double swerveDeadband = 0.1;
    //Rotate Joystick axis deadband - bigger deadband to avoid rotational drift
    public static final double swerveRotateDeadband = 0.17;
    public static final double MinGasPedalSpeed=0.30;
  }

  public static final class AutoConstants{
        public static final double pathPlanDelay=5;

        public static final double wingShotSpeed1=-50;
        public static final double wingShotSpeed2=-80;
        public static final double wingShotAngle=41;
        
        public static final double autoCloseAngle=29;
        public static final double autoCloseSpeed1=-50;
        public static final double autoCloseSpeed2=-50;

        public static final String Center="Center Start";
        public static final String LeftAmp="Left Amp";
        public static final String LeftSpeaker="Left Speaker";
        public static final String RightSpeaker="RightSpeaker";

        public enum StartingLocations {
          LEFTAMP,
          LEFTSPEAKER,
          CENTER1,
          RIGHTSPEAKER,
          TESTSTART
        }

        public enum NotePoses {
          CENTERNOTEPOS,
          AMPNOTEPOS,
          SOURCENOTEPOS,
          LONGAMP1POS,
          LONGAMP1INTAKEPOS,
          LONGAMP2POS,
          LONGCENTERPOS,
          LONGSOURCE1POS,
          LONGSOURCE2POS,
          AMPNOTEWAYPOINT,
          WINGSHOOTPOS,
          AMPSHOOTPOS,
          OUTOFBOUNDPOS,
          TWONOTEMOVEOUT,
          LONGSOURCE1WAYPOINT,
          BLUESIDEAMPREDSIDESOURCE
        }

        public enum ShootLocations{
          RIGHTSPEAKER,
          LEFTSPEAKER,
          LEFTWINGSHOOTPOS,
          RIGHTWINGSHOOTPOS,
          AMPSHOOT
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

        public static final String autoModeX0="0-Do not use";
        public static final String autoModeX1="1=Score1,Mov Out";
        public static final String autoModeX2="2=TBD";
        public static final String autoModeX3="3=TBD";
        public static final String autoModeX4="4=Wing and a Prayer";
        public static final String autoModeX5="5=Magic Time";
        public static final String autoModeX6="6=Go Robot Go";
        public static final String autoModeX7="7=maybe this times";


        public static final String autoMode0="0-Dynamic Auto Program";
        public static final String autoMode1="1=Do Nothing";
        public static final String autoMode2="2=PP-Start Center,Score, Center Short, Amp Short";
        public static final String autoMode3="3=PP-Start Center, Center Note, Amp Note, Amp Long";
        public static final String autoMode4="4=PP-Start Left, Amp Short, Amp Long";
        public static final String autoMode5="5=PP Start Right, Source Long1, SourceLong2";
        public static final String autoMode6="6=PP-Any Pose 2 Note";
        //public static final String autoMode7="7=PP-Start Center, Center Note, Amp Long";
        //public static final String autoMode8="8=PP-Start Right, Source Long";
        public static final String autoMode7="7=PP Shoot, Wait, Move";
        public static final String autoMode8="8=Legacy-Score, Straight Back Score";
      //  public static final String autoMode9="9=Legacy-Score,Wait, Cross the Line Right";
        //public static final String autoMode10="10=Legacy-Score, Center Amp Short, Amp Short";
        //public static final String autoMode11="11=TBD";
        //public static final String autoMode12="12=TBD";

        public static final String noPath="No Path";
        public static final String blueShortAmpNote="BlueShortAmpNote";
        public static final String blueShortCenterNote="BlueShortCenterNote";
        public static final String blueShortSourceNote="BlueShortSourceNote";
        public static final String blueSourceMoveOut="BlueSourceMoveOut";
        public static final String blueAmpMoveOut="BlueAmpMoveOut";
        public static final String redShortAmpNote="RedShortAmpNote";
        public static final String redShortCenterNote="RedShortCenterNote";
        public static final String redShortSourceNote="RedShortSourceNote";
        public static final String redSourceMoveOut="RedSourceMoveOut";
        public static final String redAmpMoveOut="RedAmpMoveOut";
        public static final String longAmpNote1="LongAmpNote1";
        public static final String longAmpNote2="LongAmpNote2";
        public static final String longCenterNote="LongCenterNote";
        public static final String longSourceNote1="LongSourceNote2";
        public static final String longSourceNote2="LongSourceNote1";

        public static final String testStart="TEST START";

        public static final double blueSideWingLineX=5.87;
        public static final double redSideWingLineX=10.72;

        public static final double yCoordMidpoint=5.54;

        public static final Pose2d BLUETESTSTART=new Pose2d(0,0,new Rotation2d(0));
        public static final Pose2d REDTESTSTART=new Pose2d(16,0,new Rotation2d(0));
        
        public static final Pose2d BLUETEST=new Pose2d(0,1,new Rotation2d(0));
        public static final Pose2d REDTEST=new Pose2d(15,0,new Rotation2d(0));

        public static final Pose2d redSpeakerPos=new Pose2d(16.32,5.54,new Rotation2d(0));
        public static final Pose2d blueSpeakerPos=new Pose2d(0.30,5.54,new Rotation2d(0));

        public static final Pose2d presetAutoBlueShortCenterNote=new Pose2d(2.6,5.54,new Rotation2d(0));
        public static final Pose2d presetAutoBlueShortAmpNote=new Pose2d(2.6,7.01,new Rotation2d(0));
        public static final Pose2d presetAutoBlueShortSourceNote=new Pose2d(2.6,4.09,new Rotation2d(0));
        public static final Pose2d presetAutoBlueAmpWaypoint=new Pose2d(2.6,6.3,new Rotation2d(0));

        public static final Pose2d presetAutoBlueShortAmpShootPos=new Pose2d(3.9,7.01,new Rotation2d(0));
        public static final Pose2d presetAutoRedShortAmpShootPos=new Pose2d(12.5,7.01,new Rotation2d(0));
        public static final Pose2d presetAutoBlueRightWingShootPose=new Pose2d(3.49,2.75,new Rotation2d(0));
        public static final Pose2d presetAutoRedRightWingShootPose=new Pose2d(13.67,2.48,new Rotation2d(0));
        
        public static final Pose2d presetAutoBlueTwoNoteMoveOut=new Pose2d(1.89,5.54,new Rotation2d(0));
        public static final Pose2d presetAutoRedTwoNoteMoveOut=new Pose2d(14.64,5.54,new Rotation2d(0));

        public static final Pose2d presetAutoRedShortCenterNote=new Pose2d(13.5,5.57,new Rotation2d(0));
        public static final Pose2d presetAutoRedShortSourceNote=new Pose2d(13.5,4.09,new Rotation2d(0));
        public static final Pose2d presetAutoRedShortAmpNote=new Pose2d(13.5,7.01,new Rotation2d(0));
        public static final Pose2d presetAutoRedAmpWaypoint=new Pose2d(13.5,6.3,new Rotation2d(0));

        public static final Pose2d presetAutoLongAmpNote1=new Pose2d(8.29,  7.45,new Rotation2d(0));
        public static final Pose2d presetAutoLongAmpNote2=new Pose2d(8.30,  5.75,new Rotation2d(0));
        public static final Pose2d presetAutoRedLongSourceNote1=new Pose2d(8.10,  0.69,new Rotation2d(0));
        public static final Pose2d presetAutoRedLongSourceNote2=new Pose2d(8.10,  2.45,new Rotation2d(0));
        public static final Pose2d presetAutoBlueLongSourceNote1=new Pose2d(8.70,  0.69,new Rotation2d(0));
        public static final Pose2d presetAutoBlueLongSourceNote2=new Pose2d(8.70,  2.45,new Rotation2d(0));

        public static final Pose2d locationBlueShortCenterNote=new Pose2d(2.89,5.54,new Rotation2d(0));
        public static final Pose2d blueShortCenterNoteIntakeWaypoint=new Pose2d(1.7,5.54,new Rotation2d(0));
        public static final AutoPose2D blueShortCenterAutoPoses=new AutoPose2D(locationBlueShortCenterNote,blueShortCenterNoteIntakeWaypoint,
                                                                              locationBlueShortCenterNote,false,true);
 
        public static final Pose2d locationBlueShortAmpNote=new Pose2d(3,6.99,new Rotation2d(0));
        public static final Pose2d blueShortAmpNoteIntakeWaypoint=new Pose2d(1.7,6.99,new Rotation2d(0));
        public static final AutoPose2D blueShortAmpAutoPoses=new AutoPose2D(locationBlueShortAmpNote,blueShortAmpNoteIntakeWaypoint,
                                                                            locationBlueShortCenterNote,false,true);
        
        public static final Pose2d locationBlueShortSourceNote=new Pose2d(3,4.09,new Rotation2d(0));
        public static final Pose2d blueShortSourceNoteIntakeWaypoint=new Pose2d(1.7,4.09,new Rotation2d(0));
        public static final AutoPose2D blueShortSourceAutoPoses=new AutoPose2D(locationBlueShortSourceNote,blueShortSourceNoteIntakeWaypoint,
                                                                              locationBlueShortCenterNote,false,true);

        public static final Pose2d locationRedShortCenterNote=new Pose2d(13.68,5.57,new Rotation2d(0));
        public static final Pose2d redShortCenterNoteIntakeWaypoint=new Pose2d(14.39,5.57,new Rotation2d(0));
        public static final AutoPose2D redShortCenterAutoPoses=new AutoPose2D(locationRedShortCenterNote,redShortCenterNoteIntakeWaypoint,
                                                                              locationRedShortCenterNote,false,true);
        
        public static final Pose2d locationRedShortSourceNote=new Pose2d(13.67,4.09,new Rotation2d(0));
        public static final Pose2d redShortSourceNoteIntakeWaypoint=new Pose2d(14.39,4.09,new Rotation2d(0));
        public static final AutoPose2D redShortSourceAutoPoses=new AutoPose2D(locationRedShortSourceNote,redShortSourceNoteIntakeWaypoint,
                                                                              locationRedShortCenterNote,false,true);

        public static final Pose2d locationRedShortAmpNote=new Pose2d(13.68,7.01,new Rotation2d(0));
        public static final Pose2d redShortAmpNoteIntakeWaypoint=new Pose2d(14.39,7.01,new Rotation2d(0));
        public static final AutoPose2D redShortAmpAutoPoses=new AutoPose2D(locationRedShortAmpNote,redShortAmpNoteIntakeWaypoint,
                                                                          locationRedShortCenterNote,false,true);
        public static final Pose2d locationLongAmpNote=new Pose2d(8.29,  7.45,new Rotation2d(0));
        public static final Pose2d longAmpNoteRedIntakeWaypoint=new Pose2d(9.20,  7.47,new Rotation2d(0));
        public static final Pose2d longAmpNoteBlueIntakeWaypoint=new Pose2d(7.40,  7.47,new Rotation2d(0));
        public static final AutoPose2D longAmpAutoPoses=new AutoPose2D(locationLongAmpNote,false,true);

        public static final Pose2d locationLongAmp2Note=new Pose2d(8.30,  5.75,new Rotation2d(0));
        public static final Pose2d longAmp2NoteRedIntakeWaypoint=new Pose2d(9.05,  5.75,new Rotation2d(0));
        public static final Pose2d longAmp2NoteBlueIntakeWaypoint=new Pose2d(7.66,  5.75,new Rotation2d(0));
        public static final AutoPose2D longAmp2AutoPoses=new AutoPose2D(locationLongAmp2Note,false,true);

        public static final Pose2d locationLongCenterNote=new Pose2d(8.30,  4.09,new Rotation2d(0));
        public static final Pose2d longCenterNoteRedIntakeWaypoint=new Pose2d(9.05,  4.11,new Rotation2d(0));
        public static final Pose2d longCenterNoteBlueIntakeWaypoint=new Pose2d(7.66,  4.11,new Rotation2d(0));
        public static final AutoPose2D longCenterAutoPoses=new AutoPose2D(locationLongCenterNote,false,true);

        public static final Pose2d locationLongSource2Note=new Pose2d(8.30,  2.45,new Rotation2d(0));
        public static final Pose2d longSource2NoteRedIntakeWaypoint=new Pose2d(9.05,  2.45,new Rotation2d(0));
        public static final Pose2d longSource2NoteBlueIntakeWaypoint=new Pose2d(7.66,  2.45,new Rotation2d(0));
        public static final AutoPose2D longSource2AutoPoses=new AutoPose2D(locationLongSource2Note,false,true);

        public static final Pose2d locationLongSourceNote=new Pose2d(8.24,  0.78,new Rotation2d(0));
        public static final Pose2d longSourceNoteRedIntakeWaypoint=new Pose2d(9.00, 0.69,new Rotation2d(0));
        public static final Pose2d longSourceNoteBlueIntakeWaypoint=new Pose2d(7.2, 0.69,new Rotation2d(0));
        public static final AutoPose2D longSourceAutoPoses=new AutoPose2D(locationLongSourceNote,false,true);
       
        public static final Pose2d locationRedLongAmpWing=new Pose2d(12.22,6.43,new Rotation2d(0));
        public static final Pose2d locationBlueLongAmpWing=new Pose2d(-12.22,-1.72,new Rotation2d(0));
        public static final Pose2d locationRedLongSourceWing=new Pose2d(4.45,6.43,new Rotation2d(0));
        public static final Pose2d locationBlueLongSourceWing=new Pose2d(-4.45,-1.72,new Rotation2d(0));
       
        public static final Pose2d redCenterStartPos=new Pose2d(15.12,5.54,new Rotation2d(0));
        public static final Pose2d blueCenterStartPos=new Pose2d(-1.43,-5.54,new Rotation2d(0));

        public static final Pose2d redRightAmpStartPos=new Pose2d(15.33,6.98,new Rotation2d(0));
        public static final Pose2d blueRightAmpStartPos=new Pose2d(-1.22,-6.98,new Rotation2d(0));

        public static final Pose2d redLeftSpeakerStartPos=new Pose2d(15.14,4.06,new Rotation2d(0));
        public static final Pose2d blueLeftSpeakerStartPos=new Pose2d(-1.43,-6.99,new Rotation2d(0));

        public static final Pose2d redRightSpeakerStartPos=new Pose2d(15.13,6.98,new Rotation2d(0));
        public static final Pose2d blueRightSpeakerStartPos=new Pose2d(-1.43,-4.06,new Rotation2d(0));

        public static final Pose2d redAmpMoveOutOfBoundPos=new Pose2d(12.17,6.82,new Rotation2d(0));
        public static final Pose2d blueAmpMoveOutOfBoundPos=new Pose2d(4.25,6.98,new Rotation2d(0));
        public static final Pose2d redSourceMoveOutOfBoundPos=new Pose2d(12.79,1.84,new Rotation2d(0));
        public static final Pose2d blueSourceMoveOutOfBoundPos=new Pose2d(3.86,1.50,new Rotation2d(0));
        public static final AutoPose2D moveOutRedSourceAutoPose=new AutoPose2D(redSourceMoveOutOfBoundPos,redSourceMoveOutOfBoundPos,redSourceMoveOutOfBoundPos,false,false);
        public static final AutoPose2D moveOutRedAmpAutoPose=new AutoPose2D(redAmpMoveOutOfBoundPos,redAmpMoveOutOfBoundPos,redAmpMoveOutOfBoundPos,false,false);
        public static final AutoPose2D moveOutBlueSourceAutoPose=new AutoPose2D(blueSourceMoveOutOfBoundPos,blueSourceMoveOutOfBoundPos,blueSourceMoveOutOfBoundPos,false,false);
        public static final AutoPose2D moveOutBlueAmpAutoPose=new AutoPose2D(blueAmpMoveOutOfBoundPos,blueAmpMoveOutOfBoundPos,blueAmpMoveOutOfBoundPos,false,false);
      
        public static final double distancetoShortNote=2.0;
        public static final double distancBacktoNoteLine=0.15;
        public static final double distanceBetweeNotes=1.6;
      }
    
    public static final class AutoManual{
      public static final double autoDriveSpeed=0.7;
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
      public static final double intakeExtensionSpeed=1;
      public static final int intakeBreakBeamport=9;
      public static final int intakeMotorPort1=18;
      public static final int intakeMotorPort2=19;
      public static final int intakeExtensionPort=15;

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
      public static final double softLimitRightLow=-20.2;
      public static final double softLimitLeftHigh=80;
      public static final double turretDeadBand=0.5;

      public static final double turretStraightPos=0;
      public static final double turret90Pos=36.8;
      public static final double turretNonCenterShootPos=20.4;
      public static final double turretAmpNoteShootPos=15;
      public static final double turretWingShotPos=16;

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

   public static final class NoteFlipConstants{
    public static final int canID=12;
    public static final double noteFlipSpeed=-0.75;//.35 on 3/25
    public static final double startNoteFlipSpeed=-0.3;
    public static final double timeToRunAmpScore=1;
   }

   public static final class LEDConstants{
    public static final int ledPWM=9;
    public static final int ledCount=25;

    public static final double timeToWait=3;

    public static final double blinkinGreen=0.71;
    public static final double blinkinYellow=0.69;
    public static final double blinkinRed=0.61;

    public static final int candleId=14;
    public static final int candleLEDCount=180;

    public static final int candleRobotFrontStartIndex=0;
    public static final int candleRobotFrontLEDCount=21;
    public static final int candleRobotFrontSecondStartIndex=147;
    public static final int candleRobotFrontLEDCount2=34;
    public static final int candleTargetLockIndex1=106;
    public static final int candleLEDTargetLockLength1=40;
    public static final int candleTargetLockIndex2=22;
    public static final int candleLEDTargetLockLength2=40;
    public static final int candleAmpScoreIndex=61;
    public static final int candleLEDAmpScoreLength=45;

    public static enum PrimaryLEDStates{
      AUTO,TELEOP,TARGETLOCK,OFF,NOTARGET
    }

    public static enum CANDLELEDStates{
      TARGETLOCK,NOTARGET,ROBOTFRONTSIDE,AMPSCORESIDE,DISABLED,NOSTATE
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

   public static final class BlinkinConstants{
    public static final int pwmID=9;

    public static final double red=0.61;
    public static final double orange=0.65;
    public static final double green=0.35;
    public static final double purple=0.91;
    public static final double blue=0.87;

     public static enum LEDStates{
      AUTO,NOTARGET,ISATLIMIT,INTAKEHASNOTE,TARGETLOCK,OFF
    }

    public static enum LEDColors{
      GREEN,RED,ORANGE,BLUE,PURPLE
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
      public static final double closeToMediumXCheck=6;
      public static final double medXCheck=16;
      public static final double farXCheck=25;

      public static final double XCheck1=1.5;
      public static final double XCheck2=3;
      public static final double XCheck3=5; 
      public static final double XCheck4=7;
      public static final double XCheck5=9;
      public static final double XCheck6=13;
      public static final double XCheck7=18;
      public static final double XCheck8=25;

      public static final double limeLightDeadBand=0.7;
      public static final double limelightLastDisDeadband=0.1;

      public static final double limeLightTrackSpeed0=0.03;
      public static final double limeLightTrackSpeed1=0.05;
      public static final double limeLightTrackSpeed2=0.08;
      public static final double limeLightTrackSpeed3=0.10;
      public static final double limeLightTrackSpeed4=0.5; //0.35
      public static final double limeLightTrackSpeed5=0.8;

      public static final double limeLightTrackSpeed0Normal=0.03;
      public static final double limeLightTrackSpeed1Normal=0.05;
      public static final double limeLightTrackSpeed2Normal=0.10;
      public static final double limeLightTrackSpeed3Normal=0.13;
      public static final double limeLightTrackSpeed4Normal=0.15;
      public static final double limeLightTrackSpeed5Normal=0.20;
      public static final double limeLightTrackSpeed6Normal=0.25;
      public static final double limeLightTrackSpeed7Normal=0.5; //0.35
      public static final double limeLightTrackSpeed8Normal=0.8;

      public static final String turretLimelightName="";
      public static final String staticLimelightName="static-limelight";

      public static final double podiumShotDis=0;
      public static final double midShotDis=0;
      public static final double shortShotDis=0;
      public static final double limelightDisDeadband=0.45;

      public static final int redSpeakerPipeline=0;
      public static final int blueSpeakerPipeline=1;
  }
   public static final class CAN {
    public static final String CanivoreBus="5454Canivore";
   }
    public static final class ShooterConstants{
      public static final double kAngleDeadband=0.3;
      public static final double kIntakeFeedAngleDeadband=10;
      public static final double kRampUpTime=1;
      public static final double kAccelCompFactor = 0.100; // in units of seconds
      public static final double autoShooterSpeed=1;
      public static final double testShooterSpeed1=-80;
      public static final double shooterIntakeSpeed=10;
      public static final double feederSpeed=-1;
      public static final double intakeFeederSpeed=-0.4;
      public static final double ampScoreSpeed=-7.5; //-7.5 3/24
      public static final double manualfeederSpeed=-0.3;
      public static final double baseSpeedDeadband=2;
      //public static final double baseMotorSpeed=-20; //rps
      public static final double baseMotorSpeed=0; //rps
      public static final double primeMotorSpeed=-20; //rps
      public static final double timeToRunShooter=1;
      public static final double rotateSpeed=0.99;
      public static final double rotateSlowSpeed=0.3;

      public static final double homePos=72.86;
      public static final double homeDeadband=0.05;
      public static final double homeRotateSpeed=0.05;
      public static final double rotateLowSoftLimit=-43.69;
      public static final double rotateHighSoftLimit=58.78; 

      public static final double podiumShotEncoderVal=-66;
      public static final double midShotEncoderVal=0;
      public static final double shortShotEncoderVal=-39;
      public static final double shooterPosDeadband=0.15;

      public static final int shooterMotorPort1=24;
      public static final int shooterMotorPort2=22;
      public static final int feederMotorPort=21;
      public static final int shooterAnglePort=23;
      public static final int encoderCanID=11;
      public static final double shooterStowAngle=58.78;
      public static final double shooterVisionClearanceAngle=35;
      public static final double shooterVisionClearanceHighPoint=35;
      public static final double shooterAmpScoreAngle=-26; //-29.5
      public static final double shooterSourceIntakeAngle=55.28;
    }


    public static final class LimitSwitches {
      public static final int brakeButtonPort=0;
    }

    public static final class ButtonBindings{
      public static final int driverintakeToggleButtonIn=1;
      //public static final int driverturret90=2;
      //public static final int driverturret0=3;
      public static final int drivermanualShootButton=2;
      public static final int driverintakeToggleButtonOut=4;
      public static final int driverTurretToggle=5;
      public static final int driverStow=6;
      public static final int driverGyroResetButton=7;

      public static final int driverturretPOVLeft=270;
      public static final int driverturretPOVRight=90;

      public static final int operatorintakeToggleButtonIn=1;
      public static final int operatorNoteFlip=2;
      public static final int operatorintakeConveyButtonIn=3;
      public static final int operatorintakeToggleButtonOut=4;
      public static final int operatorTurretToggle=5;
      public static final int operatorStow=6;
      //public static final int operatorShooterIntake=6;
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

      public static final int customManual=3;
      public static final int customShot1=10;//4
      public static final int customShot2=5;
      public static final int customShot3=6;
      public static final int customShot4=4;//7
      public static final int customShot5=8;
      public static final int customShot6=9;
      public static final int customShot7=7;//10
      public static final int customShot8=11;
      public static final int customShot9=12;
    }
    public static final double customShot1Velocity1=-40;
    public static final double customShot1Velocity2=-40;
    public static final double customShot1Angle=21.5;
 
    public static final double customShot2Velocity1=-45;
    public static final double customShot2Velocity2=-48;
    public static final double customShot2Angle=-27.8;
 
    public static final double customShot3Velocity1=-48;
    public static final double customShot3Velocity2=-51;
    public static final double customShot3Angle=31.4;

    public static final double customShot4Velocity1=-30;
    public static final double customShot4Velocity2=-30;
    public static final double customShot4Angle=-30;
    
    public static final double customShot5Velocity1=-30;
    public static final double customShot5Velocity2=-30;
    public static final double customShot5Angle=-30;
    
    public static final double customShot6Velocity1=-30;
    public static final double customShot6Velocity2=-30;
    public static final double customShot6Angle=-30;

    public static final double notePass10Velocity1=-40;
    public static final double notePass10Velocity2=-40;
    public static final double notePass10Angle=-24;

    public static final double notePass11Velocity1=-50;
    public static final double notePass11Velocity2=-50;
    public static final double notePass11Angle=-24;

    public static final double notePass12Velocity1=-60;
    public static final double notePass12Velocity2=-60;
    public static final double notePass12Angle=-24;
    

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
          {10.0,-40,-40,21.5,1,1,2},  
          {17.4,-40,-40,21.5,1,1,2},
          {19.1,-40,-40,22.5,5,1,4},
          {22.8,-45,-48,27.5,5,1,5},
          {24.9,-45,-48,30.5,5,1,5},
          {27.1,-45,-48,31.5,5,1,7},
          {29.4,-48,-51,33,5,1,7},
          {31.4,-48,-51,34.4,5,1,7},
          {34.1,-49,-54,36,5,1,7},
          {35.6,-49,-54,37,5,1,7},
          {37.1,-51,-56,37.3,5,1,7},
          {39,-53,-60,37.6,5,1,7},
          {40.2,-65,-70,37.9,5,1,7},
          {43.9,-44,-75,38.6,5,1,7},
          {44.1,-45,-75,38.8,5,1,7},
          {45.4,-45,-75,39,5,1,7},
          {46.5,-45,-75,39.5454,5,1,7},
          {48.9,-46,-76,39.6,5,1,7},
          {51.2,-50,-80,40,5,1,7},
          {53.4,-50,-80,40.6,5,1,7},
          {55.5,-50,-80,40.9,5,1,7},
          {57.1,-50,-80,41.2,5,1,7},
          {57.3,-50,-80,41.3,5,1,7},
          {90.1,-58,-88,45,5,1,10} // dummy value
         
      };
       public static final double [][]olddistanceLookup={
           //distance , spe65555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555588ed, angle, shottime
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
