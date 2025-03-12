 package frc.robot;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

public final class Constants {
    public static final int p = 0;
    public static final int i = 0;
    public static final int d = 0;
    public static final int k30Amp = 30;
    public static final class ChargedUp {
        public static final double targetHeightAprilTag=18;     // middle of April Tag in Distance
        public static final double targetHeightPlayerStationTag=26.55;  
        public static final double targetHeighMLowTape=24.125;  // middle of low tape in inches
        public static final double targetHeightHighTape=43.875; // middle of high tape in inches 
        public static final int GridPosUpperLeft=1;
        public static final int GridPosMiddleLeft=2;
        public static final int GridPosBottomLeft=3;
        public static final int GridPosUpperCenter=4;
        public static final int GridPosMiddleCenter=5;
        public static final int GridPosBottomCenter=6;
        public static final int GridPosUpperRight=7;
        public static final int GridPosMiddleRight=8;
        public static final int GridPosBottomRight=9;
        public static final int GridPosUpperConeAny=10;
        public static final int GridPosMiddleConeAny=11;
        public static final int GridPosBottomConeAny=12;
        public static final int GridPosUpperCubeAny=13;
        public static final int GridPosMiddleCubeAny=14;
        public static final int GridPosBottomCubeAny=15;
        public static final int playerStation=16;

        public static final int Cone=0;
        public static final int Cube=1;

        public static final double leftTargetPositionX = 24;
        public static final double middleTargetPositionX = 0;
        public static final double rightTargetPositionX = -15;
        public static final double distanceFromTag = 2.8;
        public static final double AprilTagAlignmentToleranceX=1.5;
        public static final double AprilTagAlignmentToleranceArea=.25;
    }   
    
    
   
    public class swerveDrive{
        public static final double driveDeadband=0.05;
        public static final int kFrontLeftSteering = 1;
        public static final int kFrontRightSteering = 3;
        public static final int kBackLeftSteering = 2;
        public static final int kBackRightSteering = 0;
        public static final int kFrontLeftDrive = 7;
        public static final int kFrontRightDrive = 4;
        public static final int kBackLeftDrive = 5;
        public static final int kBackRightDrive = 6;
    }



    public static final class Rotate{
        public static final double rotateAutoOutStage1Speed=-0.50;
        public static final double rotateAutoOutStage2Speed=-0.13; 
        public static final double rotateAutoInSpeed=0.8; 
        public static final double angleLowConeStage1ABS=0.497;    
        public static final double angleLowConeStage2ABS=0.484;    
        public static final double angleMiddleConeStage1ABS=0.50;    
        public static final double angleMiddleConeStage2ABS=0.50; //Absolute value for mid cone --JACKSON CHNG0
        public static final double angleHighConeStage1ABS=0.510;    
        public static final double angleHighConeStage2ABS=0.495;
        public static final double angleLowCubeStage1ABS=0.497;    
        public static final double angleLowCubeStage2ABS=0.484; 
        public static final double angleMiddleCubeStage1ABS=0.510;    
        public static final double angleMiddleCubeStage2ABS=0.495; 
        public static final double angleHighCubeStage1ABS=0.510;    
        public static final double angleHighCubeStage2ABS=0.482;
        public static final double anglePlayerStage1ABS=0.536;
        public static final double anglePlayerStage2ABS=0.536;
        public static final double angleSlideStage1ABS=0;
        public static final double angleSlideStage2ABS=0;
        public static final double angleIntakePos=0.02;
        public static final double ABSHomePos = 0.536;
        public static final double homeTimeFailsafe=5;
        public static final double homeSpeedForward=0.06;
        public static final double homeSpeedBackward=-0.06;
        public static final double encoderLowScorePos=0;
    }


    public static final class ButtonConstants{
        public static final int DriverIntakeIn=1;
        public static final int DriverIntakeOut=3;
        public static final int OperatorIntakeOut=4;
        public static final int DriverInclineUp=5;
        public static final int DriverInclineDown=6;
        public static final int DriverShoot = 2;
        public static final int DriverSetInclineLow = 6;
        public static final int DriverSetInclineHigh = 5;
        public static final int DriverSetInclineMiddle = 7;
        public static final int DriverClimberUpPOV=0;
        public static final int DriverClimbDownPOV=180;
        public static final int OperatorShootClose = 2;
        public static final int OperatorShootHigh = 3;
        public static final int OperatorShootLow= 11;
        public static final int OperatorShootManual=8;
        public static final int OperatorIntakeIn=7;
    }


    public static final class AutoConstants {
        public static final String autoMode0="0-Do Nothing";
        public static final String autoMode1="1=Score";
        public static final String autoMode2="2=Score / Move";
        public static final String autoMode3="3-Blue amp / Score / Move / Score short amp";
        public static final String autoMode4="4-Blue source / Score / Move / Score short source";
        public static final String autoMode5="5-Red amp / Score / Move / Score short amp";
        public static final String autoMode6="6-Red source / Score / Move / Score short source";
       
        public static final String delayMode0="0 Seconds";
        public static final String delayMode1="3 Seconds";
        public static final String delayMode2="5 Seconds";
        public static final String delayMode3="8 Seconds";
        public static final int delayValMode0=0;
        public static final int delayValMode1=3;
        public static final int delayValMode2=5;
        public static final int delayValMode3=8;
        public static final int defaultDelayMode=0;
        public static final double MoveSpeed=0.3;
        //128 was last match

        public static final double shoot0Speed = 0.5;
        public static final double shoot1Speed = 1;

        public static final double inclineSet0 = 0.5;
        public static final double inclineSet1 = 1;

 
    }
            
        
    public static final class InputControllers {
        public static final int kXboxDrive = 0;
        public static final int kXboxOperator = 1;
        public static final int kCustomController = 2;
    }

    /*
     * Constant Values for Limelight based on target and mounting
     */
    public class LEDS {
        public static final int UPPERPORT = 0;
        public static final int BOTTOMPORT = 9;
        public static final int UPPERCOUNT = 178;
        public static final int BOTTOMCOUNT = 20;
        public static final int FLASH_DELAY=5;

        public class Colors {
            // public static final int RED = 0;
            public static final int PINK = 1;
            public static final int PURPLE = 2;
            public static final int BLUE = 3;
            public static final int CYAN = 4;
            public static final int GREEN = 5;
            public static final int YELLOW = 6;
                public static final int ORANGE = 7;
        }
    }
       
    public static class RobotMap {
        public static final int DRIVETRAIN_FRONT_LEFT_ANGLE_MOTOR = 41; // CAN 12 
        public static final int DRIVETRAIN_FRONT_LEFT_ANGLE_ENCODER = 0;//1;; // Analog3
        public static final int DRIVETRAIN_FRONT_LEFT_DRIVE_MOTOR = 31; // CAN 1
    
        public static final int DRIVETRAIN_FRONT_RIGHT_ANGLE_MOTOR = 42; // CAN 13
        public static final int DRIVETRAIN_FRONT_RIGHT_ANGLE_ENCODER =1;//2; // Analog 0 
        public static final int DRIVETRAIN_FRONT_RIGHT_DRIVE_MOTOR = 32; // CAN 32
    
        public static final int DRIVETRAIN_BACK_RIGHT_ANGLE_MOTOR = 43; // CAN 21 
        public static final int DRIVETRAIN_BACK_RIGHT_ANGLE_ENCODER =2; // Analog 1 
        public static final int DRIVETRAIN_BACK_RIGHT_DRIVE_MOTOR = 33; // CAN 22
    
        public static final int DRIVETRAIN_BACK_LEFT_ANGLE_MOTOR = 44; // CAN 10
        public static final int DRIVETRAIN_BACK_LEFT_ANGLE_ENCODER = 3; // Analog 2
        public static final int DRIVETRAIN_BACK_LEFT_DRIVE_MOTOR = 34; // CAN 36
    }
    
   public static final class PIDSteering{
    public static final double rightKP=-0.05;
    public static final double leftKP=0.05;
    public static final double rightKI=-0;  
    public static final double leftKI=0;
    public static final double rightKD=0.0001;
    public static final double leftKD=0.0001;
    public static final double forwardKP= -0.45;
    public static final double backwardKP = 0.45;
    public static final double forwardKI = -0;
    public static final double backwardKI = 0;
    public static final double forwardKD = -0;
    public static final double backwardKD = 0;
   }

    public static final PPHolonomicDriveController pathPlanDriveController = new PPHolonomicDriveController(
    new PIDConstants(5.0, 0, 0), // Translation constants 
    new PIDConstants(25.0, 0, 1) // Rotation constants
    );

   public static final class WPISwerve{
    public static final double wheelDiameter=1;
    public static final double driveGearRatio=1;
    public static final double turningGearRatio=1;

    public static final double absoluteEncoderOffsetRad=1.0;
    public static final double physicalMaxSpeedMetersPerSecond=5;
    public static final double physicalMaxAngularSpeedRadiansPerSecond=2*2*Math.PI;
    public static final double driveEncoderRot2Meter=driveGearRatio*Math.PI*wheelDiameter;
    public static final double driveEncoderRPM2MPS=driveEncoderRot2Meter/60;
    public static final double turningEncoderRot2Meter=turningGearRatio*2*Math.PI;
    public static final double KTurningEncoderRPM2MPS=turningEncoderRot2Meter/60;

    public static final double maxSwerveSpeedMetersSecond=physicalMaxSpeedMetersPerSecond/8;
    public static final double maxSwerveAngularSpeedRadianPerSecond=physicalMaxSpeedMetersPerSecond/15;
   }
// AdvantageKit Constants
public static final Mode currentMode = Mode.REAL;

public static enum Mode {
  /** Running on a real robot. */
  REAL,

  /** Running a physics simulator. */
  SIM,

  /** Replaying from a log file. */
  REPLAY
}
}