/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import frc.robot.commands.TurnRobot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static final class InputControllers{
        public static final int kJoystickLeft=1;
        public static final int kJoystickRight=2;
        public static final int kXboxMain=0;
	}
    public static final class DriveConstants {
        public static final int kLeftMotor1Port = 0;
        public static final int kLeftMotor2Port = 1;
        public static final int kRightMotor1Port = 2;
        public static final int kRightMotor2Port = 3;
    
       // public static final int[] kLeftEncoderPorts = new int[]{0, 1};
       // public static final int[] kRightEncoderPorts = new int[]{2, 3};
       // public static final boolean kLeftEncoderReversed = false;
       // public static final boolean kRightEncoderReversed = true;
    
       // public static final int kEncoderCPR = 1024;
       // public static final double kWheelDiameterInches = 6;
       // public static final double kEncoderDistancePerPulse =
            // Assumes the encoders are directly mounted on the wheel shafts
        //    (kWheelDiameterInches * Math.PI) / (double) kEncoderCPR;
      }
   public static final class IntakeConstants {
       public static int intakePort=4;
       public static double intakeSpeed=0.90;
   }
   public static final class ElevatorConstants {
    public static int elevatorLowPort=5;
    public static int elevatorHighPort=6;
   }
   public static final class ShooterConstants {
    public static int shooterTopPort=1;
    public static int shooterBottomPort=2;
   }
   public static final class LiftConstants {
    public static int downLift=7;
    public static int upLift=3;
   }
   public static final class DriveMode {
       public static int forwardMode=0;
       public static int reverseMode=1;
   }
   public static final class ShooterSpeeds{
    public static double ShooterSpeed10=-.45;
    public static double ShooterSpeed20=-.65;

   }
   public static final class LiftSpeeds{
       public static double kLliftUpNormal=-0.80;
       public static double kLiftUpMedium=-0.40;
       public static double kLiftUpSlow=-0.20;
       public static double kLiftDownNormal=0.80;
       public static double kLiftDownMedium=0.40;
       public static double kLiftDownSlow=0.20;
   }
   public static final class ButtonConstants {
       public static int elevatorCombinedUp=1;
       public static int elevatorCombinedDown=2;
       public static int ShooterSpin=2;
       public static int ShooterCombo10=3;
       public static int ShooterCombo20=4;
       public static int TestTurnLeft=3;
       public static int TestTurnRight=4;
       public static int ShooterMulti10=14;
       public static int intakeInMain=1;
       public static int xboxElevatorCombinedUp=5;  
       public static int xboxElevatorCombinedDown=6;
       public static int xboxIntakeIn=9;
       public static int xboxIntakeOut=10;
       public static int xboxComboLift=8;
       public static int xboxUpLiftUp=4;
       public static int xboxUpLiftDown=1;
       public static int xboxDownLiftUp=3;
       public static int xboxDownLiftDown=2;

       
      
       public static int intakeIn1=5;
       public static int intakeIn2=6;
       public static int intakeIn3=7;
       public static int intakeIn4=10;
       public static int intakeIn5=9;
       public static int intakeOut=8;
       public static int elevatorIn1=13;
       public static int elevatorIn2=12;
       public static int elevatorIn3=11;
       public static int elevatorOut1=14;
       public static int elevatorOut2=15;
       public static int elevatorOut3=16;
       public static int Shooter1=98;
       public static int Shooter2=98;
       public static int Shooter3=98;
       
   }
   public static final class AutoModes{
       public static Integer autoNothing=0;
       public static Integer autoShoot=1;
       public static Integer autoShootandMove=2;
       public static Integer autoMove=3;
       public static Integer autoDelayShoot=4;
       public static Integer autoDelayShootandMove=5;
       public static Integer autoDelayMove=6;
   }
   public static final class AutoNav{
    public static double autoDriveSpeed=0.2;
    public static double autoTurnSpeed;
    public static double autoTurnAngle;
   }
}