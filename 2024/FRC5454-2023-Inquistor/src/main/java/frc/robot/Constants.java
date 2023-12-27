 
package frc.robot;
import edu.wpi.first.networktables.Publisher;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DigitalInput;

public final class Constants {
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

    public static final class VisionPipelines{
        public static final int AprilTag=0;
        public static final int TopTape=1;
        public static final int BottomTape=2;
        public static final int PlayerStationTag=3;
    }

    public static enum TargetHeight
    {
                    TOPCONE,MIDDLECONE,BOTTOMCONE,TOPCUBE,MIDDLECUBE,BOTTOMCUBE,PLAYERSTATION,SLIDE;	
    }
    
    public class Spindexer {
     //   public static final int motorPort=17;
     // haha failed thing
        public static final double spinForward=1;
        public static final double spinBack=-1;
        public static final double spinForwardSlow=0.6;
        public static final double spinBackSlow=-0.6;
    }    
    public class Elevator{
        public static final int elevatorPort=19;
        public static final double elevatorSpeed=.50;
        public static final double elevatorLimitSpeed=.50;
        public static final double homeTimeOut=.6;
        public static final int limitSwitch=3;
        public static final double maxLimit = -153;
    }
    public class Claw{
        public static final int clawMotorPort=24;
        public static final double clawSpeed=1;
    }
    public class RotateArm{
        public static final int rotateArmPort1=50;
        public static final int rotateArmPort2=51;
        public static final int absoluteEncoder=0;
        public static final double manualSpeed=0.4;
        public static final double manualLimitSpeed=0.4;
        public static final double encodervalueHomePos=.536;
        public static final double encoderFrontLimit=.415;
        public static final double encoderBackLimit=.613;
        
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


    public static final class Lift{
        public static final double liftAutoExtendStage1Speed=-0.9; //-0.5
        public static final double liftAutoExtendStage2Speed=-0.4; //-0.3
        public static final double liftAutoRetract1Speed=0.9;
        public static final double liftAutoRetract2Speed=0.6;
        public static final double liftAutoRetractHomeSpeed=0.1;
        public static final double kClawFlipOffset=6; // add a positive to decrease the stroke since it is negative values;
        //low pos meeds to be higher so cones are near spindexer
        public static final double posInitLift=-70-kClawFlipOffset;
        //all other positions go lower
        public static final double posCubeOutofLimelight=-60; 
        public static final double posLiftOutfIntake=-25; 
        public static final double posLiftCubeTransfer=-25;
        public static final double posInitLiftRetract=-46+kClawFlipOffset; // was -26+kClawFip
         public static final double posLowConeFullLiftStage1=-62+kClawFlipOffset;
        public static final double posLowConeFullLiftStage2=-72+kClawFlipOffset;
        public static final double posMiddleConeFullLiftStage1=-80;
        public static final double posMiddleConeFullLiftStage2=-92; //mid cone auto for lift --JACKSON CHNG
        public static final double posHighConeFullLiftStage1=-70; //was 95
        public static final double posHighConeFullLiftStage2=-133; //was -136
        public static final double posLowCubeFullLiftStage1=-62+kClawFlipOffset;
        public static final double posLowCubeFullLiftStage2=-72+kClawFlipOffset;
        public static final double posMiddleCubeFullLiftStage1=-70.00+kClawFlipOffset;
        public static final double posMiddleCubeFullLiftStage2=-82; 
        public static final double posHighCubeFullLiftStage1=-90+kClawFlipOffset;
        public static final double posHighCubeFullLiftStage2=-110;
        public static final double posPlayerLiftStage1=-55;  
        public static final double posPlayerLiftStage2=-83; //WAS -76 --JACKSON
        public static final double posSlideStage1=-70;   
        public static final double posSlideStage2=-79; 
        public static final double posShelf=-98+kClawFlipOffset;
        public static final double posHome=10;
        
        public static final double liftKP = 0.1;
        public static final double liftKI = 1e-4;
        public static final double liftKD = 1;
        public static final double liftKIZ = 0;
        public static final double liftKFF = 0;
        public static final double maxOutput = 1;
        public static final double minOutPut = -1;

        public static final int topTape = 0;
        public static final int middleTape = 1;
        public static final int apriltag = 2;
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

    public static final class FloorIntake{
        public static final double intakeSpeed=.9;
        public static final double intakeRotateSpeed=0.5;
     
        public static final double intakehomeRotateSpeed=-0.15;
        public static final int intakeMotorPort1=44;
        public static final int intakeMotorPort2=17;
        public static final int rotateMotorPort=42;
        public static final double rotateHighLimit=2;
        public static final double rotateLowLimit=130;
     
        public static final double rotateOuttakePos=98;
        
        public static final double rotateHandOff=30;
        public static final int intakeRotateEncoderPort=4;
        public static final double homeTimeOut=2;
        public static final double autoTimeOut=5;
    }

    public static final class Pneumatics {
        public static final int CompressorID=0; 
        public static final int HubID=62;
        public static final PneumaticsModuleType moduleType=PneumaticsModuleType.REVPH;
    
        public static final int clawSolenoid = 9; // was 8 -jackson
        public static final int punchSolenoid=8; //was 9 -jackson
    }

    public static final class ButtonConstants{
        public static final int DriverSolenoidPunch=1;
        public static final int DriverAutoBalance=2;
        public static final int DriverIntakeIn=3;
        public static final int DriverDownSpeed=5;
        public static final int DriverUpSpeed=6;
        public static final int DriverDriveMode=7;
        public static final int DriverGyroReset=8;
        
        public static final int OperatorWrist=1;
        public static final int OperatorRetractElevator=2;
        public static final int OperatorClawSwap=3;
        public static final int OperatorPlayerStation=4;
        public static final int OperatorAutoLow=5;
        public static final int OperatorAutoMiddle=6;
        public static final int OperatorIntakeIn=7;
        public static final int OperatorIntakeOut=8;
     
        public static final int OperatorPOVRotateUp=0;
        public static final int OperatorPOVRotatedown=180;
        public static final int OperatorPOVIntakeIn=270;
        public static final int OperatorPOVIntakeOut=90;
       
        public static final int CustomCtlRetract=3;
        public static final int TargetTopLeft=4;
        public static final int TargetMiddleLeft=7;
        public static final int TargetBottomLeft=10;
        public static final int TargetTopCenter=5;
        public static final int TargetMiddleCenter=8;
        public static final int TargetBottomCenter=11;
        public static final int TargetTopRight=6;
        public static final int TargetMiddleRight=9;
        public static final int TargetBottomRight=12;

        public static final double JoystickDeadBand = 0.10;
        public static final double ElevatorDeadBand = 0.10;
        public static final double RotateDeadBand = 0.10;
        public static final double IntakeDeadBand = 0.30;
        public static final double LeftTriggerDeadBand = 0.05;
        public static final double RightTriggerDeadBand = 0.05;
    }

    public static final class LimitSwitches{
        public static final int brakeButtonPort=2;
    }

    public static final class AutoModes {
        public static final String autoMode0="0-Do Nothing";
        public static final String autoMode1="1=Score";
        public static final String autoMode2="1=Score / Move out of Zone";
        public static final String autoMode3="1-Score / Engage Charging Station";
        public static final String autoMode4="1-Score / Move Out Of Zone / Engage Charging Station";
        public static final String autoMode5="2-Score / Leave / Get Cone / Score";

        public static final int autoNothing = 0;
        public static final int autoMoveBack = 1;
        public static final int autoScoreLeave = 2;
        public static final int autoScoreDock = 3;
        public static final int autoScoreEngage = 4;
        public static final int autoScore2=5;
        
        public static final String delayMode0="0 Seconds";
        public static final String delayMode1="3 Seconds";
        public static final String delayMode2="5 Seconds";
        public static final String delayMode3="8 Seconds";
        public static final int delayValMode0=0;
        public static final int delayValMode1=3;
        public static final int delayValMode2=5;
        public static final int delayValMode3=8;
        public static final int defaultDelayMode=0;
        public static final double MoveSpeed=0.5;
        //128 was last match
        public static final double LeaveCommunityDistance=155;//115; // 120  // 15 feet = 174-30
        public static final double pushDistance = 5;
        public static final double DistanceToCharging=20;// x feet
        public static final double DistanceToDock=30;
    }
            
        
    public static final class InputControllers {
        public static final int kXboxDrive = 0;
        public static final int kXboxOperator = 1;
        public static final int kCustomController = 2;
    }
    public static final class PhotonVision{
        public static String camera="";
    }

    /*
     * Constant Values for Limelight based on target and mounting
     */
    public static final class LimeLightValues {
        public static final double steeringP = 0.035;
        public static final double steeringI = 0;
        public static final double steeringD = 0.0055;
        public static final double steeringFeedForward = 0.0;

        public static final double targetHeight = 18; // 249 cm
        public static final double targetXPosShoot = -1.5;
        public static final double targetXPosSafeZone = 5;
        public static final double targetXPosRange=50;
        public static final double limelightHeight = 19; //37
        public static final double limelightAngle = 0; //40
        public static final double kVisionDistanceTolerance = 5;
        public static final double kVisionXTolerance = .04;
        public static final double kVisionXOffset=4;
        public static final double kVisionXMaxDistanceOffset=4.31; // was 1.7
        public static final double kVisionXMinDistanceOffset=0.91; // was 1.7
    }

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
        public static final int DRIVETRAIN_FRONT_LEFT_ANGLE_MOTOR = 20; // CAN
        public static final int DRIVETRAIN_FRONT_LEFT_ANGLE_ENCODER = 2;//1;; // Analog
        public static final int DRIVETRAIN_FRONT_LEFT_DRIVE_MOTOR = 25; // CAN
    
        public static final int DRIVETRAIN_FRONT_RIGHT_ANGLE_MOTOR = 21; // CAN
        public static final int DRIVETRAIN_FRONT_RIGHT_ANGLE_ENCODER =1;//2; // Analog
        public static final int DRIVETRAIN_FRONT_RIGHT_DRIVE_MOTOR = 26; // CAN
    
        public static final int DRIVETRAIN_BACK_RIGHT_ANGLE_MOTOR = 27; // CAN
        public static final int DRIVETRAIN_BACK_RIGHT_ANGLE_ENCODER = 0; // Analog
        public static final int DRIVETRAIN_BACK_RIGHT_DRIVE_MOTOR = 22; // CAN
    
        public static final int DRIVETRAIN_BACK_LEFT_ANGLE_MOTOR = 28; // CAN
        public static final int DRIVETRAIN_BACK_LEFT_ANGLE_ENCODER = 3; // Analog
        public static final int DRIVETRAIN_BACK_LEFT_DRIVE_MOTOR = 23; // CAN
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