package frc.robot;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.config.PIDConstants;
import edu.wpi.first.math.Matrix;

public final class Constants {
  public static final double kRobotLoopTime = 0.020;
  public static final int k30Amp=30;
  public static final int k20Amp=20;
  public static final int k26Amp=26;
  public static final int k15Amp=15;
  public static final int k40Amp=40;
  public static final int k70Amp=70;
  public static final int k80Amp=80;
  public static final int pdhCAN=1;

  public static final Matrix<N3, N1> kPoseEstimatorStandardDeviations = VecBuilder.fill(0.1, 0.1, 10);
  public static final Matrix<N3, N1> kVisionStandardDeviations = VecBuilder.fill(5, 5, 500);

  //home time out
  public static final int homeTimeOut=3;
  
  public static final class DriveConstants{
    // Joystick axis deadband for the swerve drive
    public static final double swerveDeadband = 0.1;
    //Rotate Joystick axis deadband - bigger deadband to avoid rotational drift
    public static final double swerveRotateDeadband = 0.17; //0.17
    public static final double MinGasPedalSpeed=0.20;
    //Support    6328 DriveConstants Class
    public static final double trackWidthX = edu.wpi.first.math.util.Units.inchesToMeters(27.5);
    public static final double  trackWidthY= edu.wpi.first.math.util.Units.inchesToMeters(27.5);
  
    public static final Translation2d[] moduleTranslations = {
    new Translation2d(trackWidthX / 2, trackWidthY / 2),
    new Translation2d(trackWidthX / 2, -trackWidthY / 2),
    new Translation2d(-trackWidthX / 2, trackWidthY / 2),
    new Translation2d(-trackWidthX / 2, -trackWidthY / 2)
  };

  }

  public static final class AutoConstants{
   
  }

  
  public static final class InputControllers {
    public static final int kXboxDrive=0;
    public static final int kXboxOperator=1;
    public static final int kCustomController=2;
    public static final int kFunnyController=3;
    public static final double kRumbleoff=0;
    public static final double kRumbleLight=0.25;
    public static final double kRumbleMedium=0.5;
    public static final double kRumbleFull=1.0;
  }

  public static final class LimeLightValues {
    public static final double leftLimelightHeight=0.2205;
    public static final double leftLimelightAngle=30;
    public static final double backLimelightHeight=0.2205;
    public static final double backLimelightAngle=30; 
    
    public static final double turretLimelightHeight  =17.75;
    public static final double turretLimelightAngle=24;
    public static final String turretLimelightName="limelight-three";
    
    public static final String leftLimelightName="limelight-two";
    public static final String backLimelightName="limelight-one";

    //Confidence Max and Min Deadband Values
    public static final double confidenceDeadbandMin=60;
    public static final double confidenceDeadbandMax=140;

    //Multiplier to Turn Pathplanner X & Y Pos to Meters
    public static final double cartPointToMeterMult=1;

    public static final double maxMeterDiff=0.1;
    public static final double maxRotDiff=5;

    //Tested Average of Differences in X coords
    public static final double confidenceXMean=0.017692936661984578;
    //Tested Average of Differences in Y coords
    public static final double confidenceYMean=1.2286761570822624E-5;

    public static final double driveDeadBand=2;

    public static final double isInDeadBandHeartBeat=2;
    public static final double shouldEndHeartBeat=6;
    public static final double strafeClampMinLEFT=0.029;
    public static final double strafeClampMinRIGHT=0.029;

    public static final double strafePRIGHT=0.007;
    public static final double strafeIRIGHT=0;
    public static final double strafeDRIGHT=0.02;
    public static final double strafeMaxAndMinRIGHT=0.1;
    public static final double strafeInputGainRIGHT=1;

    public static final double strafePLEFT=0.007;
    public static final double strafeILEFT=0;
    public static final double strafeDLEFT=0.02;
    public static final double strafeMaxAndMinLEFT=0.20;
    public static final double strafeInputGainLEFT=1;

    public static final double rotP=0.07;
    public static final double rotI=0;
    public static final double rotD=0.2;
    public static final double rotMaxAndMin=0.2;
    public static final double rotInputGain=60;

    public static final double correctDriveSpeed=-0.1;

    public static final double isInDeadBandHeartBeatNEW=2;
    public static final double shouldEndHeartBeatNEW=4;
    public static final double strafeClampMinLEFTNEW=0.03;
    public static final double strafeClampMinRIGHTNEW=0.03;
    public static final double strafePassOff=0.15;

    public static final double strafePRIGHTNEW=0.012;
    public static final double strafeIRIGHTNEW=0.0000000001;
    public static final double strafeDRIGHTNEW=9;
    public static final double strafeMaxAndMinRIGHTNEW=0.45;
    public static final double strafeInputGainRIGHTNEW=0.5;

    public static final double strafePLEFTNEW=0.01;
    public static final double strafeILEFTNEW=0.0000000001;
    public static final double strafeDLEFTNEW=11;
    public static final double strafeMaxAndMinLEFTNEW=0.45;
    public static final double strafeInputGainLEFTNEW=0.5;

    public static final double driveP=0.15;
    public static final double driveI=0;
    public static final double driveD=0.1;
    public static final double driveMaxAndMin=0.5;
    public static final double driveInputGain=0.5;
  }

  public static final class LocationConstants{
     public static final Pose2d redHub= new Pose2d(12.5051566,3.6657534,Rotation2d.fromDegrees(0));
     public static final Pose2d blueHub= new Pose2d(5.21534,3.6657534,Rotation2d.fromDegrees(0));
     
    }
  public static final class IntakeConstants{
    public static final int IntakeMotorCanID=59;
    public static final int FoldMotorCanID=24;
    public static final int intakeSwitchDIO=6;
    public static final double lowSpeed=-0.5;
    public static final double highSpeed=-1.0;
    public static final double outtakeSpeed=0.9;
    public static final double foldSpeed=-0.5;//-0.2;
    public static final double foldHomeSpeed=-0.15;
    public static final double foldSpeedAutoMode=-0.15;
    public static final double ampInStop=30;//10;
    public static final double ampOutStop=80;//20;
    public static final double intakeinPos=0.05;
    public static final double intakeRollStop=9;
    public static final double intakeEndStop=14.2; 

  }

  public static final class PassConstants{    
      public static enum PassTargets{
          FARLEFT,LEFT,MIDDLE,RIGHT,FARRIGHT
      }
  }
  public static final class ShooterConstants {
    public static final int shooter1CANID=29;
    public static final int shooter2CANID=28;
    public static final int hoodCANID=27;
    public static final int kickerCANID=61;
    public static final int fuelSensorDIO=9;
    //fixed shots distances
    public static final double fixedShotDistance1=73;
    public static final double fixedShotDistance2=97;
    public static final double fixedShotDistance3=133;
    public static final double kDefaultShootingDistance=60;
    public static final double kDefaultPassDistance=200;
    //speeds
    public static final double shooterRPM=5000;
    public static final double IdleSpeed=55;// 0.7;
    public static final double KickerSpeed=-1; //pos 1 for B-bot
    public static final double shootSpeed=100; //0.8;
    public static final double kAgitateTimeLimit=5; //how long to agitate on empty
    public static final double shooterVelocityLowDeadband=0.1;
    public static final double shooterVelocityHighDeadband=1;
    //6328 Transforms for Turret
    public static Transform3d robotToTurret = new Transform3d(-0.19685, 0.0, 0.44, Rotation3d.kZero);
    public static Transform3d turretToCamera =
      new Transform3d(
          -0.1314196, 0.0, 0.2770674, new Rotation3d(0.0, Units.degreesToRadians(-22.5), 0.0));
  }
  public static final class HopperConstants{
    public static final int HopperMotorCanID=60;
    public static final double agitateSpeed=1;
  }
  public static final class HoodConstants{
    public static final int hoodCoderCANID=51;
    public static final double hoodOffset=-0.080000;//-0.488037109375;
    public static final double hoodSpeed=0.06;
    public static final double hoodUpSpeed=-0.06;
    public static final double hoodDownSpeed=0.06;
    public static final double hoodDeadband=0.02;
  }
  public enum TurretStates {MANUAL,TRACK,SEARCH,FIXEDLEFT,FIXEDRIGHT,ODOM,END};
  public enum TurretTrackingMethod {HUB,PASS,TURRET,NOTARGET};
  public static final class TurretConstants {
    public static final int turretCanID=6;
    public static final double turretSpeed=0.11; 
    public static final int TurretPOT=2;
    public static final int TurretPOTZero=0;
    public static final double TurretLeftLimitPOTS=0.15; 
    public static final double TurretRightLimitPOTS=0.85;
    public static final int encoder1CANID=11;
    public static final int encoder2CANID=12;
    public static final double kTrackingSpeed=0.05454;
    public static final double trackerDeadBand =1.0;

    public static final double kTurretOffsetX=-0.0762;
    public static final double kTurretOffsetY=0.1397;
    public static final double kMinShootingDistance=0;
    public static final double kMaxShootingDistance=100;
    public static final double kMinAngleDegrees=0;
    public static final double kMaxAngleDegrees=300;
    public static final double REDUCTION = 13.2;
    public static final double MIN_ROT_DEG = -360.0;
    public static final double MAX_ROT_DEG = 360.0;
    public static final double CRT_MATCH_TOLERANCE = 0.20;//0.01;
    
    public static final int GEAR_0_TOOTH_COUNT = 100;
    public static final int GEAR_1_TOOTH_COUNT = 40;
    public static final int GEAR_2_TOOTH_COUNT = 41;

            // Added absolute turret system constants
        public static final double MOTOR_TO_TURRET_RATIO = 45.7143;
        public static final double ENC1_RATIO = 8.0;
        public static final double ENC2_RATIO = 320.0 / 41.0;
        public static final double ABSOLUTE_SOLVE_STEP_DEG = 0.25;
        public static double ENC1_FORWARD_OFFSET_DEG = 62.0;
        public static double ENC2_FORWARD_OFFSET_DEG = 175.0;
        public static final boolean ENC1_INVERT = false;
        public static final boolean ENC2_INVERT = false;
  }

  public static final class ClimbConstants{
    public static final int climbCanID1=43;
    public static final int climbUpSwitchDIO=8;
    public static final int climbDownSwitchDIO=7;
    public static final double climbP=0.2;
    public static final double climbI=0;//DONT USE
    public static final double climbD=0;
    public static final double climbMaxAndMin=1;
    public static final double climbInputGain=100;

    public static final class AutoAlign{
      public static final double climbYpos=00;
      public static final double climbXpos=00;
      public static final double alignRotation=00;

      public static final double yTolerance=05;
      public static final double ySetpoint=00;
      public static final double xTolerance=05;
      public static final double xSetpoint=00;
      public static final double rotTolerance=03;
      public static final double rotSetpoint=00;

      public static final double waitTime=5;
      public static final double validationTime=4;
    }

    public static final double climbPos1=0.28;
    public static final double climbPos2=0.49;

    public static final double climbLimitLow=0.0; // was 0.17 
    public static final double climbLimitHigh=0.49;
    public static final double climbForwardSpeed=-1.0;
    public static final double climbBackSpeed=1.0;
  }

  public static final class CoolPanelConstants{
    public static final int greenPWM=0;
    public static final int redPWM=1;
  }

  public final class LedConstants{
    public static final int LedCanID = 14;
    public static final int LedCount = 115;
    public static final int larsonSize=1;
  }

  public static enum LEDStates{
      ENABLED, DISABLED, INTAKING, TELEOP, AUTOSCORING, LINEDUP,DISABLEDERROR,DISABLEDSEETARGET
  }

  public static enum ColorStates{
      YELLOW, WHITE, PURPLE, GREEN, RED, BLUE
  }

  public static enum AnimationStates{
      FIRE,RAINBOW,PURPLELARSON,REDSTROBE,REDLARSON,GREENFLASHING,PURPLEFLASHING,NULL
  }

  public static final class ButtonBindings{
    public static final int button=1;

    public static final double joystickDeadband = 0.1;
  }

  public static final class OperatorConstants {
    public static final double kDeadband = 0.01;
    public static final double kCubic = 0.95;
    public static final double kLinear = 0.05;
  }

  public static enum NotificationLevel {
    INFO, WARNING, ERROR, QUESTION
  }

  public static final PPHolonomicDriveController pathPlanDriveController = new PPHolonomicDriveController(
    new PIDConstants(3.0, 0, 0.25), // Translation constants 
    new PIDConstants(25.0, 0, 1) // Rotation constants
  );

  //SUPPORT for 6328 functions - imported from 2026 project
  public static boolean disableHAL = false;

  public static void disableHAL() {
    disableHAL = true;
  }
  
  public static final double loopPeriodSecs = 0.02;
  public static final double loopPeriodWatchdogSecs = 0.2;
 //End of 6328 Support
}
