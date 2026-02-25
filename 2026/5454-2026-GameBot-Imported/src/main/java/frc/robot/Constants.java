package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
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
  public static final int k80Amp=80;
  public static final int pdhCAN=1;
  
  public static final class DriveConstants{
    // Joystick axis deadband for the swerve drive
    public static final double swerveDeadband = 0.1;
    //Rotate Joystick axis deadband - bigger deadband to avoid rotational drift
    public static final double swerveRotateDeadband = 0.17; //0.17
    public static final double MinGasPedalSpeed=0.20;
    //Support for 6328 DriveConstants Class
    public static final double trackWidthX = 20.75;
    public static final double  trackWidthY= 20.75;
  
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

  public static final class IntakeConstants{
    public static final int IntakeMotorCanID=24;
    public static final int LowMotorCanID=59;
    public static final double lowSpeed=-0.5;
    public static final double highSpeed=-0.9;
    public static final double outtakeSpeed=0.9;
  
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
    public static final int fuelSensorDIO=1;
    //speeds
    public static final double shooterRPM=5000;
    public static final double IdleSpeed=0.5;
    public static final double KickerSpeed=1;
    public static final double shootSpeed=1;
    //6328 Transforms for Turret
    public static Transform3d robotToTurret = new Transform3d(-0.19685, 0.0, 0.44, Rotation3d.kZero);
    public static Transform3d turretToCamera =
      new Transform3d(
          -0.1314196, 0.0, 0.2770674, new Rotation3d(0.0, Units.degreesToRadians(-22.5), 0.0));
  }
  public static final class HopperConstants{
    public static final int HopperMotorCanID=60;
    public static final double agitateSpeed=-1;
  }
  public static final class HoodConstants{
    public static final double hoodUpSpeed=-0.06;
    public static final double hoodDownSpeed=0.06;
  }
  public static final class TurretConstants {
    public static final int turretCanID=6;
    public static final double turretSpeed=0.1;
    public static final int TurretPOT=0;
    public static final int encoder1CANID=11;
    public static final int encoder2CANID=12;
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
        public static double ENC1_FORWARD_OFFSET_DEG = 0.0;
        public static double ENC2_FORWARD_OFFSET_DEG = 0.0;
        public static final boolean ENC1_INVERT = false;
        public static final boolean ENC2_INVERT = false;

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
    new PIDConstants(5.0, 0, 0), // Translation constants 
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
