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
  public static final int brakeButton=3;
  public static final int pdhCAN=1;
  
  public static final class DriveConstants{
    // Joystick axis deadband for the swerve drive
    public static final double swerveDeadband = 0.1;
    //Rotate Joystick axis deadband - bigger deadband to avoid rotational drift
    public static final double swerveRotateDeadband = 0.12; //0.17
    public static final double MinGasPedalSpeed=0.20;
  }

  public static final class LineupConstants{
    public static final Pose2d fiducial17LeftReef=new Pose2d(3.955,3.438,new Rotation2d().fromDegrees(-120));
    public static final Pose2d fiducial17RightReef=new Pose2d(4.234,3.284,new Rotation2d().fromDegrees(-120));

    public static final Pose2d fiducial18LeftReef=new Pose2d(3.695,4.179,new Rotation2d().fromDegrees(180));
    public static final Pose2d fiducial18RightReef=new Pose2d(3.695,3.861,new Rotation2d().fromDegrees(180));

    //Changed by one in direction of origin
    public static final Pose2d fiducial19LeftReef=new Pose2d(3.253,3.756,new Rotation2d().fromDegrees(120));
    public static final Pose2d fiducial19RightReef=new Pose2d(2.964,3.593,new Rotation2d().fromDegrees(120));
 
    public static final Pose2d fiducial20LeftReef=new Pose2d(5.023,4.593,new Rotation2d().fromDegrees(60));
    public static final Pose2d fiducial20RightReef=new Pose2d(4.724,4.756,new Rotation2d().fromDegrees(60));
 
    public static final Pose2d fiducial21LeftReef=new Pose2d(5.27,3.85,new Rotation2d().fromDegrees(0));
    public static final Pose2d fiducial21RightReef=new Pose2d(5.27,4.180,new Rotation2d().fromDegrees(0));

    public static final Pose2d fiducial22LeftReef=new Pose2d(4.734,3.284,new Rotation2d().fromDegrees(-60));
    public static final Pose2d fiducial22RightReef=new Pose2d(5.003,3.438,new Rotation2d().fromDegrees(-60));

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

    public static final double lineUpDeadband=0.05;
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
    public static final double limelightNeuralHeight=12; //37
    public static final double limelightNeuralAngle=-10; //40
    public static final double limelightBackOdomHeight=5;
    public static final double limelightBackOdomAngle=0;
    public static final double limelightFrontOdomHeight=3;
    public static final double limelightFrontOdomAngle=0;

    public static final String frontOdomLimelightName="limelight-odom";
    public static final String backOdomLimelightName="limelight-odombwd";
    public static final String neuralLimelightName="limelight-neural";

    public static final int centerApriltagPipeline=0;
    public static final int leftApriltagPipeline=1;
    public static final int rightApriltagPipeline=2;

    //Confidence Max and Min Deadband Values
    public static final double confidenceDeadbandMin=60;
    public static final double confidenceDeadbandMax=140;

    //Multiplier to Turn Pathplanner X & Y Pos to Meters
    public static final double cartPointToMeterMult=0.5;

    public static final double maxMeterDiff=0.1;
    public static final double maxRotDiff=5;

    //Tested Average of Differences in X coords
    public static final double confidenceXMean=0.017692936661984578;
    //Tested Average of Differences in Y coords
    public static final double confidenceYMean=1.2286761570822624E-5;

    public static final double xLineupDeadband=20;
    public static final double yawLineupDeadband=45;

    public static final double driveDeadband0=30;
    public static final double driveDeadband1=60;
    public static final double driveDeadband2=150;
    public static final double driveDeadband3=200;

    public static final double reefAprilTagHeight=8.75;

    public static final double odomLineUpXOffsetCenter=0.7;
    public static final double odomLineUpYOffsetCenter=0;
    public static final double odomLineUpXOffsetLeft=0.7;
    public static final double odomLineUpYOffsetLeft=0.3;
    public static final double odomLineUpXOffsetRight=0.7;
    public static final double odomLineUpYOffsetRight=-0.3;

    public static enum LimelightLineUpOffsets{
      LEFT,CENTER,RIGHT
    }
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

    public static final double posDeadband=1.2;

    public static final double homePosDeadband=0.001;
    public static final double rotateHomePos=0.18;
    public static final double homeSpeed=0.03;

    public static final double localPIDkP=0.1;
    public static final double localPIDkI=0;
    public static final double localPIDkD=0;
    public static final double localPIDMaxAndMin=0.5;

    public static final double l1PosABS=0;
    public static final double l2PosABS=0;
    //8.5 inches from reef to robot frame
    public static final double l3PosABS=0;
    //14 inches from reef to robot frame
    public static final double l4PosABS=0;
    public static final double humanPlayerPosABS=0;

    public static final double relativeHighLimitABS=0.48;
    public static final double relativeLowLimitABS=0.10; //no limit 

    public static final double autoScoreAlgaeSpeed=1;
    public static final double autoScoreCoralSpeed=1;
    public static final double autoTimeToRun=1.3;
  }

  public static final class ElevatorConstants{
    public static final int elevatorCanID=21;
    public static final int canAndColorID=0;

    public static final double elevatorPK1=0.05;//0.25
    public static final double elevatorIK1=0;
    public static final double elevatorDK1=0;
    public static final double elevatorMaxAndMinK1=1;

    public static final double elevatorPK2=0.05;//0.25
    public static final double elevatorIK2=0;
    public static final double elevatorDK2=0;
    public static final double elevatorMaxAndMinK2=0.50;

    public static final double elevatorHighLimit=-99.5;
    public static final double elevatorLowLimit=-0.2;

    public static final double posDeadband=1.2;

    public static final double l1Pos=-13;
    public static final double l2Pos=-14; //26
    public static final double l3Pos=-43;
    public static final double l4Pos=-89; //99.5

    public static final double aboveTroughPos=-20;

    public static enum ElevatorScoreLevel{
      L1,L2,L3,L4,RETRACT, TEST
    }
  }

  public static final class ClimbConstants{
    public static final int climbCanID1=26;
    public static final int climbCanID2=20;
    public static final int encoderDIO=2;

    public static final double climbP=0.2;
    public static final double climbI=0;//DONT USE
    public static final double climbD=0;
    public static final double climbMaxAndMin=1;
    public static final double climbInputGain=100;

    public static final double climbPos1=0.28;
    public static final double climbPos2=0.49;

    public static final double climbLimitLow=0.0; // was 0.17 
    public static final double climbLimitHigh=0.49;
  }

  public static final class CoolPanelConstants{
    public static final int greenPWM=0;
    public static final int redPWM=1;
  }

  public final class LedConstants{
    public static final int LedCanID = 14;
    public static final int LedCount = 300;
  }

  public static enum LEDStates{
      TELEOP, DISABLED, HASCORAL, HASALGEA, LINEDUP, SCORED, ATHUMANPLAYER
  }

  public static enum ColorStates{
      GREEN, PURPLE, RED, BLUE, WHITE
  }

  public static enum AnimationStates{
      FIRE, RAINBOW, LARSON, NULL
  }

  public static final class ButtonBindings{
    public static final int dunkinCoralOutakeButton=2; 
    public static final int dunkinCoralIntakeButton=4;
    public static final int lineUpButton=5;
    public static final int elevatorScoreLevelButton=6;

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
