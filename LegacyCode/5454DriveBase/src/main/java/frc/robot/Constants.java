// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final double kSpeedMultiplier = 0.65;
    
    public static final class DriveConstants {
        public static final int kLeftMotor1Port = 10;
        public static final int kLeftMotor2Port = 11;
        public static final int kRightMotor1Port = 12;
        public static final int kRightMotor2Port = 13;
    }
    public static final class InputControllers{
        public static final int kJoystickLeft=0;
        //public static final int kJoystickRight=1;
    }
}
