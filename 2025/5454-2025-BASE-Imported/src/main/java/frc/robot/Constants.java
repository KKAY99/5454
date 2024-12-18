 
package frc.robot;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.wpilibj.XboxController;
import com.pathplanner.lib.auto.AutoBuilder;

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
      public static final double limeLightTrackSpeed5=0.6;

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

      public static final int driverPOVLeft=270;
      public static final int driverPOVRight=90;
      public static final int driverPOVUp=0;
      public static final int driverPOVDown=180;

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
      public static final int customShot1=4;
      public static final int customShot2=5;
      public static final int customShot3=6;
      public static final int customShot4=7;
      public static final int customShot5=8;
      public static final int customShot6=9;
      public static final int customShot7=10;
      public static final int customShot8=11;
      public static final int customShot9=12;
    }
    public static final double customShot1Velocity1=-25;
    public static final double customShot1Velocity2=-25;
    public static final double customShot1Angle=21.5;
 
    public static final double customShot2Velocity1=-30;
    public static final double customShot2Velocity2=-40;
    public static final double customShot2Angle=37.9;

    public static final double customShot3Velocity1=-40;
    public static final double customShot3Velocity2=-50;
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

    public static final double customShotAngleDEMO=21.5;
    

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
    }

    public static final PPHolonomicDriveController pathPlanDriveController = new PPHolonomicDriveController(
      new PIDConstants(5.0, 0, 0), // Translation constants 
      new PIDConstants(5.0, 0, 0) // Rotation constants
    );
}
