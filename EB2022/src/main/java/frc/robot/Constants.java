/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.io.File;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final double kSpeedMultiplier=.80;  // change to fraction to slow robot down
    public static final double kHookLiftTime=2.5;
    public static final double kSlowMoveLeft=0.25;
    public static final double kSlowMoveRight=0.20;
    public static final double kSlowMoveTurn=0.35;
    public static final class LimitSwitches{
        public static final int ArmDown=0;
        public static final int ArmUp=1;

    }
    public static final class InputControllers{
        public static final int kJoystickLeft=0;
       // public static final int kJoystickRight=1;
        public static final int kXboxMain=1;
        public static final int kXboxPit=3;
	}
    public static final class DriveConstants {
        public static final int kLeftMotor1Port = 1;
        public static final int kLeftMotor2Port = 2;
        public static final int kRightMotor1Port = 8;
        public static final int kRightMotor2Port = 9;
    }
   public static final class IntakeConstants {
       public static int intakeLiftMotorPort=4;
       public static int intakeMotorPort=0;
    
       public static double intakeSpeed=0.95;
   }
   public static final class ClimberConstants {
       public static int climberMotorPort=3;
   }
   public static final class DriveMode {
       public static int forwardMode=0;
       public static int reverseMode=1;
   }
   public static final class Pneumatics {
       public static int CompressorNode=0;
       public static int LatchPort=0;
   }

   
   public static final class IntakeLiftSpeeds{
       public static double intakeLiftUpSpeedSlow=.65;
       public static double intakeLiftDownSpeedSlow=.2;
   }
   public static final class ClimberSpeeds{
       public static double ClimberSpeedSlow =0.80;
       public static double ClimberSpeedFast =0.99;
       public static double ClimberSpeedBack =-0.80;
       
       

   }  
   public static final class ColorWheel{
       public static int wheelMotorPort=5;
       public static double RightSpeed=.9;
       public static double LeftSpeed=-.9;
   }
   public static final class ButtonConstants {
     public static int intakeIn=1;//joystick controls   
     public static int intakeOut=2;
     public static int intakeLiftUp=3;
     public static int intakeLiftDown=4;
     public static int climberSlow=10;
     public static int climberFast=9;
     public static int climberTime=8;
     public static int CameraSwitch=9;
     public static int climberBackJS=7;
     
     //back should be 7 and slow should be 1 - testing

     public static int climberBackXB=1;//xbox controls
     public static int climberSlowXB=4;
     public static int telopAutoShoot=2;
     public static int intakeInXB=5;   
     public static int intakeOutXB=6;
     public static int FlipLatchXB=8;

     public static int climberFastXB=4;
     public static int climberTimeXB=3;
    public static int intakeLiftUpXB=3;
     public static int intakeLiftDownXB=4;
     
     public static int ReverseDriveMode=8;
    }
    public static final class LimeLightValues{
        public static final double targetHeight=98.03; // 249 cm
        //KK FRONt LIMELIGH public static final double limelightHeight=21.5;
      //KK FRONt LIMELIGH  public static final double limelightAngle=14.5;
      public static final double limelightAngle=45;
      public static final double limelightHeight=22.875;
    }

   public static final class AutoModes{
       public static final int autoNothing=0;
       public static final int autoMoveForward=1;
       public static final int autoMoveBackward=2;
       public static final int autoMoveShoot=3;
       public static final int autoMoveToShoot=5;
       public static final int autoMoveShootFromRight=6;
   }
   public static final class AutoConstants{
       public static double moveSpeed=0.28; // adjust on carpet
       public static double moveLeftSpeed=-0.25;
       public static double moveRightSpeed=0.28;
       public static double moveTurnSpeed=0.45;
       public static double moveOnlyTime=2;
       public static double moveGoalTime=4.0;
       public static double moveRightStep1=0.4;
       public static double moveRightStep2=1.0;
       public static double moveRightStep3=2.5;
       public static double moveRightStep4=0.85;
       public static double moveRightStep5=2.0;    
       
       public static double moveLastGoalTime=4.0;
       public static double moveOffWall=1;
       public static double moveBackup=3;
   }
   public static final class TeleOpAutomationConstants{
    public static double moveSpeed=0.50; // adjust on carpet
    public static double moveLeftSpeed=-0.50;
    public static double moveRightSpeed=0.56;
    public static double moveOnlyTime=2;
    public static double moveGoalTime=1.0;
    public static double moveLastGoalTime=0.6;
    public static double moveOffWall=1;
    public static double moveBackup=3;
}


}