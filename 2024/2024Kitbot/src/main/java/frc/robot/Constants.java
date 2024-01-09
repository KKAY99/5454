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
    public static final int leftMotor1Port=0;
    public static final int leftMotor2Port=1;
    public static final int rightMotor1Port=2;
    public static final int rightMotor2Port=3;
  }

  public static final class ShooterConstants{
    public static final int shootingMotor1Port=0;
    public static final int shootingMotor2Port=1;

    public static final double shooterSpeed=1;
  }

  public static final class IntakeConstants{
    
  }

  public static final class ButtonConstants{
    public static final int xBoxDriverPort=0;
    
    public static final int shooterButton=0;
  }
}
