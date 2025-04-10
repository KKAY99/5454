package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.config.PIDConstants;

public final class Constants {
  public static final double kRobotLoopTime = 0.020;
  public static final int k30Amp=30;
  public static final int k20Amp=20;
  public static final int k26Amp=26;
  public static final int k15Amp=15;
  public static final int k40Amp=40;
  public static final int k80Amp=80;
  public static final int brakeButton=3;
  public static final int pdhCAN=1;
  
  public static final class DriveConstants{
    // Joystick axis deadband for the swerve drive
    public static final double swerveDeadband = 0.1;
    //Rotate Joystick axis deadband - bigger deadband to avoid rotational drift
    public static final double swerveRotateDeadband = 0.12; //0.17
    public static final double MinGasPedalSpeed=0.20;
  }

  public static final class AutoConstants{
    public static final double redDoSiDoDeadBand=3;
  }

  public static final class LineupConstants{
    public static final Pose2d fiducial17LeftReef=new Pose2d(3.955,3.438,new Rotation2d().fromDegrees(-120));
    public static final Pose2d fiducial17RightReef=new Pose2d(4.234,3.284,new Rotation2d().fromDegrees(-120));

    public static final Pose2d fiducial18LeftReef=new Pose2d(3.695,4.179,new Rotation2d().fromDegrees(180));
    public static final Pose2d fiducial18RightReef=new Pose2d(3.695,3.861,new Rotation2d().fromDegrees(180));

    public static final Pose2d fiducial19LeftReef=new Pose2d(3.63,5.2,new Rotation2d().fromDegrees(120));
    public static final Pose2d fiducial19RightReef=new Pose2d(3.19,5.05,new Rotation2d().fromDegrees(120));
    public static final Pose2d fiducial19LeftLineupReef=new Pose2d(3.35,5.65,new Rotation2d().fromDegrees(120));
    public static final Pose2d fiducial19RightLineupReef=new Pose2d(3.03,3.36,new Rotation2d().fromDegrees(120));
 
    public static final Pose2d fiducial20LeftReef=new Pose2d(4.762,5.05,new Rotation2d().fromDegrees(60));
    public static final Pose2d fiducial20RightReef=new Pose2d(4.45,5.12,new Rotation2d().fromDegrees(60));
    public static final Pose2d fiducial20LeftLineupReef=new Pose2d(4.98,5.35,new Rotation2d().fromDegrees(60));
    public static final Pose2d fiducial20RightLineupReef=new Pose2d(4.74,5.61,new Rotation2d().fromDegrees(60));
 
    public static final Pose2d fiducial21LeftReef=new Pose2d(5.27,3.900,new Rotation2d().fromDegrees(0));
    public static final Pose2d fiducial21RightReef=new Pose2d(5.27,4.249,new Rotation2d().fromDegrees(0));
    public static final Pose2d fiducial21LeftLineupReef=new Pose2d(6.27,3.900,new Rotation2d().fromDegrees(0));
    public static final Pose2d fiducial21RightLineupReef=new Pose2d(6.27,4.249,new Rotation2d().fromDegrees(0));

    public static final Pose2d fiducial22LeftReef=new Pose2d(4.734,3.284,new Rotation2d().fromDegrees(-60));
    public static final Pose2d fiducial22RightReef=new Pose2d(5.003,3.438,new Rotation2d().fromDegrees(-60));

    public static final Pose2d[] fiducialBlueLeftLineupPoses=new Pose2d[]{
      fiducial17LeftReef,
      fiducial18LeftReef,
      fiducial19LeftLineupReef,
      fiducial20LeftLineupReef,
      fiducial21LeftLineupReef,
      fiducial22LeftReef
    };
    
    public static final Pose2d[] fiducialBlueRightLineupPoses=new Pose2d[]{
      fiducial17RightReef,
      fiducial18RightReef,
      fiducial19RightLineupReef,
      fiducial20RightLineupReef,
      fiducial21RightLineupReef,
      fiducial22RightReef
    };

    public static final Pose2d[] fiducialBlueLeftPoses=new Pose2d[]{
      fiducial17LeftReef,
      fiducial18LeftReef,
      fiducial19LeftReef,
      fiducial20LeftReef,
      fiducial21LeftReef,
      fiducial22LeftReef
    };
    
    public static final Pose2d[] fiducialBlueRightPoses=new Pose2d[]{
      fiducial17RightReef,
      fiducial18RightReef,
      fiducial19RightReef,
      fiducial20RightReef,
      fiducial21RightReef,
      fiducial22RightReef
    };

    public static final Pose2d[] fiducialRedLeftPoses=new Pose2d[]{};
    public static final Pose2d[] fiducialRedRightPoses=new Pose2d[]{};

    public static final double lineUpDeadband=0.09;
    public static final double maxWaitTime=5;
  }
  
  public static final class InputControllers {
    public static final int kXboxDrive=0;
    public static final int kXboxOperator=1;
    public static final int kCustomController=2;
    public static final double kRumbleoff=0;
    public static final double kRumbleLight=0.25;
    public static final double kRumbleMedium=0.5;
    public static final double kRumbleFull=1.0;
  }

  public static final class LimeLightValues {
    public static final double leftLimelightHeight=0;
    public static final double leftLimelightAngle=0;
    public static final double rightLimelightHeight=0;
    public static final double rightLimelightAngle=0;

    public static final String leftLimelightName="limelight-left";
    public static final String rightLimelightName="limelight-right";

    public static final int centerApriltagPipeline=0;
    public static final int leftApriltagPipeline=1;
    public static final int rightApriltagPipeline=2;

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

    public static final double reefAprilTagHeight=8.75;

    public static final double leftLineupXDeadband=0.7;
    public static final double rightLineupXDeadband=1;

    public static final double driveTargetDistanceRight=51;
    public static final double driveTargetDistanceLeft=-66;

    public static final double driveFollowDistanceRight=100;
    public static final double driveFollowDistanceLeft=-120;
    public static final double driveDeadBand=2;

    public static final double strafePRIGHT=0.15;
    public static final double strafeIRIGHT=0;
    public static final double strafeDRIGHT=0.5;
    public static final double strafeMaxAndMinRIGHT=0.3;
    public static final double strafeInputGainRIGHT=1;

    public static final double strafePLEFT=0.15;
    public static final double strafeILEFT=0;
    public static final double strafeDLEFT=0.5;
    public static final double strafeMaxAndMinLEFT=0.3;
    public static final double strafeInputGainLEFT=1;

    public static final double rotP=0.1;
    public static final double rotI=0;
    public static final double rotD=0.2;
    public static final double rotMaxAndMin=0.5;
    public static final double rotInputGain=60;

    public static final double algaeDriveTimeToRun=0.8;
    public static final double driveTimeToRun=0.5;
    public static final double algaeDriveBackSpeed=0.4;
    public static final double lineUpDriveSpeed=-0.45454;

    public static final double driveP=0.15;
    public static final double driveI=0;
    public static final double driveD=0.1;
    public static final double driveMaxAndMin=0.5;
    public static final double driveInputGain=0.5;
  }

  public static final class DunkinDonutConstants{
    public static final int indexerLimitSwitchDIO = 4;
    public static final int coralIndexerID = 27;
    public static final int limitSwitchDIO = 3;
    public static final int canCoderID = 59;
    public static final int coralCanID = 23;
    public static final int algaeCanID1 = 25;
    public static final int algaeCanID2 = 60; //place holder ID
    public static final int rotateCanID = 24;

    public static final double posDeadband=2;

    public static final double homePosDeadband=0.001;
    public static final double rotateHomePos=0.14;
    public static final double homeSpeed=0.03;

    public static final double coralP=0.03;
    public static final double coralI=0;
    public static final double coralD=0.1;
    public static final double coralMaxAndMin=0.05;

    public static final double clearDoorPosOut=3;
    public static final double clearDoorPosIn=2.5;
    public static final double clearDoorSpeedOut=0.1;
    public static final double clearDoorSpeedIn=-0.1;

    public static final double clawPIDkP=0.05;
    public static final double clawPIDkI=0;
    public static final double clawPIDkD=0;
    public static final double clawPIDMaxAndMin=0.5;
    public static final double clawPIDInputGain=60;

    public static final double outOfLimelightVisionPos=0.165;
    public static final double groundIntakePos=0.33;
    public static final double algaeStowPos=0.30;
    public static final double lollipopGrabPos=0.328;
    public static final double processorScorePos=0.328;
    public static final double noGrabAlgaePos=0.22;
    public static final double algaeGrabPos=0.37;

    public static final double relativeHighLimitABS=0.48;
    public static final double relativeLowLimitABS=0.10; //no limit 

    public static final double groundIntakeSpeed=1;
    public static final double lollipopGrabSpeed=1;

    public static final double algaeThrowSpeed=-1;
    public static final double processorScoreSpeed=-0.5;
    public static final double autoScoreAlgaeSpeed=1;
    public static final double autoScoreCoralSpeed=0.40;// was 1 on 3/16
    public static final double autoCoralTimeToRun=1;
    public static final double autoScoreAlgaeRunTime=1;
  }

  public static final class ElevatorConstants{
    public static final int elevatorCanID=21;
    public static final int canAndColorID=0;

    public static final double elevatorPK1=0.10;//0.07
    public static final double elevatorIK1=0;
    public static final double elevatorDK1=0;
    public static final double elevatorMaxAndMinK1=1;

    public static final double elevatorPK2=0.07;//0.05
    public static final double elevatorIK2=0;
    public static final double elevatorDK2=0;
    public static final double elevatorMaxAndMinK2=0.60;

    public static final double elevatorHighLimit=-99.5;
    public static final double elevatorLowLimit=-0.2;

    public static final double posDeadband=1.2;

    public static final double elevSafeRetractPos=-43;
    public static final double aboveThrowPos=-60;
    public static final double elevAlgeaGrabRetractPos=-20;
    public static final double groundIntakePos=-5;
    public static final double processorScorePos=-13;
    public static final double lollipopGrabPos=-13;
    public static final double l3AlgaePos=-43;
    public static final double l4AlgaePos=-70;
    public static final double l1Pos=-13;
    public static final double l2Pos=-14; //26
    public static final double l3Pos=-43;
    public static final double l4Pos=-89;

    public static final double aboveTroughPos=-20;

    public static enum ElevatorScoreLevel{
      L1,L2,L3,L4,RETRACT,TEST
    }
  }

  public static final class IntakeConstants{
    public static final double coralShootSpeed=0.6;
    public static final double coralOutakeSpeed=-0.3;
    public static final double coralIntakeSpeed=0.15;
    public static final double indexerIntakeSpeed=0.75;
    public static final double indexerOuttakeSpeed=-0.75;
  }

  public static final class ClimbConstants{
    public static final int climbCanID1=26;
    public static final int climbCanID2=20;
    public static final int encoderDIO=2;
    public static final int ServoPMW=0;

    public static final double climbP=0.2;
    public static final double climbI=0;//DONT USE
    public static final double climbD=0;
    public static final double climbMaxAndMin=1;
    public static final double climbInputGain=100;

    public static final double climbPos1=0.28;
    public static final double climbPos2=0.49;

    public static final double climbLimitLow=0.0; // was 0.17 
    public static final double climbLimitHigh=0.49;
    public static final double climbForwardSpeed=0.4;
    public static final double climbBackSpeed=-0.4;
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
      ENABLED, DISABLED, INTAKING, HASCORAL, HASCORALANDDOALGEA,TELEOP, AUTOSCORING, GOLEFT, GORIGHT, LINEDUP,DISABLEDERROR,DISABLEDSEETARGET
  }

  public static enum ColorStates{
      YELLOW, WHITE, PURPLE, GREEN, RED, BLUE
  }

  public static enum AnimationStates{
      FIRE,RAINBOW,PURPLELARSON,REDSTROBE,REDLARSON,GREENFLASHING,PURPLEFLASHING,NULL
  }

  public static final class ButtonBindings{
    public static final int dunkinCoralOutakeButton=1; 
    public static final int dunkinCoralIntakeButton=4;
    public static final int elevatorScoreAutoButton=5;
    public static final int elevatorScoreManualButton=6;
    public static final int operatorStow = 7;
    public static final int setScoreLevelL1POV0=00;
    public static final int setScoreLevelL2POV90=90;
    public static final int setScoreLevelL3POV180=180;
    public static final int setScoreLevelL4POV270=270;
    
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
    new PIDConstants(5.0, 0, 0), // Translation constants 
    new PIDConstants(25.0, 0, 1) // Rotation constants
  );
}
