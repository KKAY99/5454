 package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
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
    public static final int kXboxDrive = 0;
    public static final int kXboxOperator = 1;
    public static final int kCustomController = 2;
    public static final double kRumbleoff=0;
    public static final double kRumbleLight =0.25;
    public static final double kRumbleMedium=0.5;
    public static final double kRumbleFull=1.0;
  }

  public static final class LimeLightValues {
    public static final double steeringP = 0.035;
    public static final double steeringI = 0;
    public static final double steeringD = 0.0055;
    public static final double steeringFeedForward = 0.0;

    public static final double targetHeight = 33.75; // 249 cm
    public static final double limelightNeuralHeight = 12; //37
    public static final double limelightNeuralAngle = -10; //40
    public static final double limelightOdomHeight = 18;
    public static final double limelightOdomAngle = 0;
    public static final double kVisionDistanceTolerance = 5;
    public static final double kVisionXTolerance = .04;
    public static final double kVisionXOffset=4;

    public static final String odomLimelightName="limelight-odom";
    public static final String neuralLimelightName="limelight-neural";

    public static final int centerApriltagPipeline=0;
    public static final int leftApriltagPipeline=1;
    public static final int rightApriltagPipeline=2;

    public static final double lineupDeadband0=0.3;
    public static final double lineupDeadband1=0.7;
    public static final double lineupDeadband2=3;
    public static final double lineupDeadband3=9;
    public static final double lineupDeadband4=15;

    public static final double reefAprilTagHeight=8.75;
  }

  public static final class ButtonBindings{
  }

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
  public static final class OperatorConstants {
    public static final double kDeadband = 0.01;
    public static final double kCubic = 0.95;
    public static final double kLinear = 0.05;
  }
  public static final class VisionConstants {
    public static final double kPoseErrorAcceptance = 3.0; // How much error there can be between current stimated pose
                                                           // and vision pose in meters
  }
  public static final class Swerve {
    public static final Translation2d flModuleOffset = new Translation2d(0.4, 0.4);
    public static final Translation2d frModuleOffset = new Translation2d(0.4, -0.4);
    public static final Translation2d blModuleOffset = new Translation2d(-0.4, 0.4);
    public static final Translation2d brModuleOffset = new Translation2d(-0.4, -0.4);

    public static final double maxModuleSpeed = 4.5; // M/S
  }

  public static final PPHolonomicDriveController pathPlanDriveController = new PPHolonomicDriveController(
    new PIDConstants(5.0, 0, 0), // Translation constants 
    new PIDConstants(25.0, 0, 1) // Rotation constants
  );
}
