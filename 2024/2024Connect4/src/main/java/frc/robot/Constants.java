// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class DriveConstants {
    public static final int leftMotor1Port=10;
    public static final int leftMotor2Port=11;
    public static final int rightMotor1Port=12;
    public static final int rightMotor2Port=13;
  }

  public static final class ShooterConstants{
    public static final int shootingMotor1Port=11;
    public static final int shootingMotor2Port=16;
    public static final int feederMotorPort=0; //PWM for the Win
    public static final double feederSpeed=-1;
    public static final double shooterSpeed1=.25;
  //  public static final double shooterSpeed2=0.446;
  //  public static final double shooterSpeed3=0.450;
  //  public static final double shooterSpeed4=0.454;
  
    public static final double shooterSpeed2=8000;
    public static final double shooterSpeed3=17000;
    public static final double shooterSpeed4=19000;
  //  public static final double shooterSpeed2=1000;
  //  public static final double shooterSpeed3=500;
  //  public static final double shooterSpeed4=250;
  
  }

  public static final class IntakeConstants{
    
  }

  public static final class LaserCanConstants{
    public static final double distanceToReflector=0;
    public static final double deadBand=0;
  }

  public static final class ADABreakBeamConstants{
    public static final int breakBeamDIO=0;
  }

  public static final class ButtonConstants{
    public static final int xBoxDriverPort=0;
    
    public static final int shooterButton1=1;
    public static final int shooterButton2=2;
    public static final int shooterButton3=3;
    public static final int shooterButton4=4;
  }

  public static final class LEDConstants{
    public static final int blinkInPWM=1;
    public static final int ledPWM=8;
    public static final int ledCount=20;

    public static final double timeToWait=3;

    public static enum LEDStates{
      AUTO,TELEOP,INTAKELOW,INTAKEHASNOTE,TARGETLOCK,OFF
    }

    public static enum LEDColors{
      GREEN,RED,ORANGE,BLUE,PURPLE
    }

    public static enum LEDDisplayStates{
      FLASHING,SOLID,CHASING
    }
   }
}
