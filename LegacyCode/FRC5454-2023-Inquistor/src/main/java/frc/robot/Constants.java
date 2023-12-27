// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public class Intake{
        public static final int intakePort=90;
        public static final double intakeInSpeed=0.50;
        public static final double intakeOutSpeed=-0.50;
    }    
    public class Elevator{
        public static final int elevatorPort=91;
        public static final double elevatorSpeed=.50;
    }
    public class Claw{
        public static final int clawPort=92;
    }
    public class pivotWheelPort{
        public static final int pivotWheelPort=93;
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

    public static final class Pneumatics {
        public static final int CompressorID=0; 
             
    }
    public static final class ButtonConstants{
       public static final int DriverIntakeIn=2;
       public static final int DriverIntakeOut=3;
       public static final int DriverGyroReset=7;
       public static final int DriverDriveMode=1;
       
       public static final int OperatorIntakeIn=6;
       public static final int OperatorIntakeOut=7;
    
       public static final int OperatorTurretAxis=0;
       public static final int OperatorTurretFindAxis=1;
       public static final int OperatorClimbAxis=5;
       public static final int OperatorGyroReset=7;
    }

    public static final class LimitSwitches{
        public static final int ExampleSwitch=0;
    }

    public static final class AutoModes {
        public static final String autoMode0="0-Do Nothing";
        public static final String autoMode1="0-Move Foward Out of Zone";
        public static final String autoMode2="1=Score Cube / Move out of Zone";
        public static final String autoMode3="1=Score Cone / Move out of Zone";
        public static final String autoMode4="1-Score Cube / Dock Charging Station";
        public static final String autoMode5="1-Score Cone / Dock Charging Station";
        public static final String autoMode6="1-Score Cube / Engage Charging Station";
        public static final String autoMode7="1-Score Cone / Engage Charging Station";
        
       
        public static final int defaultAutoMode=1;

        public static final int autoNothing = 0;
        public static final int autoMoveForward = 1;
        public static final int autoCubeLeave = 2;
        public static final int autoConeLeave = 3;
        public static final int autoCubeDock = 4;
        public static final int autoConeDock = 5;
        public static final int autoCubeEngage = 6;
        public static final int autoConeEngage = 7;
        
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
        public static final double LeaveZoneDistance=30;
    }
            
        
    public static final class InputControllers {
        public static final int kXboxDrive = 0;
        public static final int kXboxOperator = 1;
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
        public static final double limelightHeight = 6.5; //37
        public static final double limelightAngle = 0; //40
        public static final double kVisionDistanceTolerance = 5;
        public static final double kVisionXTolerance = 1.5;
        public static final double kVisionXOffset=4;
        public static final double kVisionXMaxDistanceOffset=4.31; // was 1.7
        public static final double kVisionXMinDistanceOffset=0.91; // was 1.7
    }

    public class LEDS {
        public static final int PORT = 0;
        public static final int COUNT = 215;
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
       
public static class RobotMap {
    public static final int DRIVETRAIN_FRONT_LEFT_ANGLE_MOTOR = 25; // CAN
    public static final int DRIVETRAIN_FRONT_LEFT_ANGLE_ENCODER = 0; // Analog
    public static final int DRIVETRAIN_FRONT_LEFT_DRIVE_MOTOR = 20; // CAN

    public static final int DRIVETRAIN_FRONT_RIGHT_ANGLE_MOTOR = 26; // CAN
    public static final int DRIVETRAIN_FRONT_RIGHT_ANGLE_ENCODER = 1; // Analog
    public static final int DRIVETRAIN_FRONT_RIGHT_DRIVE_MOTOR = 21; // CAN

    public static final int DRIVETRAIN_BACK_RIGHT_ANGLE_MOTOR = 27; // CAN
    public static final int DRIVETRAIN_BACK_RIGHT_ANGLE_ENCODER = 2; // Analog
    public static final int DRIVETRAIN_BACK_RIGHT_DRIVE_MOTOR = 22; // CAN

    public static final int DRIVETRAIN_BACK_LEFT_ANGLE_MOTOR = 28; // CAN
    public static final int DRIVETRAIN_BACK_LEFT_ANGLE_ENCODER = 3; // Analog
    public static final int DRIVETRAIN_BACK_LEFT_DRIVE_MOTOR = 23; // CAN
   }

}
