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
    public static final double MinGasPedalSpeed=0.30;
    public static final double MinGasRotateSpeed=0.30;
  }

  public static final class AutoConstants{
    public static final String autoMode1="1=AutoDoNothing";
    public static final String autoMode2="2=PathPlanTest";
    public static final String autoMode3="3=PathfindingTest";

    public static final String centerStart="CenterStart";
    public static final String leftStart="LeftStart";
    public static final String rightStart="RightStart";

    public static final class AutoPoses{
      public static final Pose2d testStartPos=new Pose2d(0,0,new Rotation2d(0));
      public static final Pose2d centerStart=new Pose2d(7.09,4.04,new Rotation2d(0));
      public static final Pose2d leftStart=new Pose2d(7.09,6.28,new Rotation2d(0));
      public static final Pose2d rightStart=new Pose2d(7.09,1.75,new Rotation2d(0));

      public static final Pose2d testPose1=new Pose2d(0,0,new Rotation2d(0));
      public static final Pose2d testPose2=new Pose2d(0,5,new Rotation2d(0));
      public static final Pose2d testPose3=new Pose2d(5,5,new Rotation2d(0));
      public static final Pose2d testPose4=new Pose2d(5,0,new Rotation2d(0));
      public static final Pose2d testPose5=new Pose2d(0,0,new Rotation2d(0));

      public static final Pose2d pathFindingTestPose1=new Pose2d(2.184,4.04,new Rotation2d(0));
    }
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
    public static final double cartPointToMeterMult=0;

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
  }

  public static final class DunkinDonutConstants{
    public static final int CANcoderID = 0;
    public static final int coralCanID = 23;
    public static final int algaeCanID = 25;
    public static final int rotateCanID = 24;

    public static final double posDeadband=5;

    public static final double homePosDeadband = 0.1;
    public static final double rotateHomePos = 0;
    public static final double homeSpeed = 0.1;
  }

  public static final class ElevatorConstants{
    public static final int elevatorCanID = 21;
    public static final double posDeadband=1;
  }

  public static final class ButtonBindings{
    public static final int DunkinCoralButton = 0;
    public static final int DunkinAlgeaButton = 0;
    public static final double joystickDeadband = 0.1;
  }

  public static final class OperatorConstants {
    public static final double kDeadband = 0.01;
    public static final double kCubic = 0.95;
    public static final double kLinear = 0.05;
  }

  public static final PPHolonomicDriveController pathPlanDriveController = new PPHolonomicDriveController(
    new PIDConstants(5.0, 0, 0), // Translation constants 
    new PIDConstants(25.0, 0, 1) // Rotation constants
  );
}
