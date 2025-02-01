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
  public static final int k30Amp=30;
  public static final class DriveConstants {
    public static final int leftMotor1Port=10;
    public static final int leftMotor2Port=11;
    public static final int rightMotor1Port=12;
    public static final int rightMotor2Port=13;
  }

  public final class LedConstants{
    public static final int LedCanID = 14;
    public static final int LedCount = 300;

    public static enum LEDStates{
      TELEOP, DISABLED, HASCORAL, HASALGEA, LINEDUP, SCORED
    }

    public static enum ColorStates{
      GREEN, PURPLE, RED, BLUE, WHITE
    }

    public static enum AnimationStates{
      FIRE, RAINBOW, LARSON, NULL
    }
  }

  public static final class ShooterConstants{
    public static final int shootingMotor1Port=21;
    public static final int shootingMotor2Port=22;
    public static final int shootingMotor3Port=0;
    public static final int shootingMotor4Port=1;

    public static final double shooterSpeed1=0.90;
    public static final double shooterSpeed2=0.95;
    public static final double shooterSpeed3=0.98;
    public static final double shooterSpeed4=1.0;
  }

  public static final class IntakeConstants{
    
  }



  public static final class ButtonConstants{
    public static final int xBoxDriverPort=0;
    
    public static final int shooterButton1=1;
    public static final int shooterButton2=2;
    public static final int shooterButton3=3;
    public static final int shooterButton4=4;
  } 
  public static final class LimeLightValues {
    public static final double steeringP = 0.035;
    public static final double steeringI = 0;
    public static final double steeringD = 0.0055;
    public static final double steeringFeedForward = 0.0;
    public static final double kVisionXOffset=0;
    public static final double kVisionDistanceTolerance=0;
    public static final double kVisionXTolerance=0;
  }

}
