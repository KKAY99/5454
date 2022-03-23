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
    public static final int CAN_TIMEOUT_MS = 20;

    public static final double DEADBAND_RANGE = 0.05;

    public static class RobotMap {
        public static final int DRIVETRAIN_FRONT_LEFT_ANGLE_MOTOR = 25; // CAN
        public static final int DRIVETRAIN_FRONT_LEFT_ANGLE_ENCODER = 0; // Analog
        public static final int DRIVETRAIN_FRONT_LEFT_DRIVE_MOTOR = 20; // CAN
        public static final double DRIVETRAIN_FRONT_LEFT_ENCODER_OFFSET = -Math.toRadians(6.5); //161.8 // toRadians(degrees)

        public static final int DRIVETRAIN_FRONT_RIGHT_ANGLE_MOTOR = 26; // CAN
        public static final int DRIVETRAIN_FRONT_RIGHT_ANGLE_ENCODER = 1; // Analog
        public static final int DRIVETRAIN_FRONT_RIGHT_DRIVE_MOTOR = 21; // CAN
        public static final double DRIVETRAIN_FRONT_RIGHT_ENCODER_OFFSET = -Math.toRadians(56.2); // toRadians(degrees)

        public static final int DRIVETRAIN_BACK_RIGHT_ANGLE_MOTOR = 27; // CAN
        public static final int DRIVETRAIN_BACK_RIGHT_ANGLE_ENCODER = 2; // Analog
        public static final int DRIVETRAIN_BACK_RIGHT_DRIVE_MOTOR = 22; // CAN
        public static final double DRIVETRAIN_BACK_RIGHT_ENCODER_OFFSET = -Math.toRadians(307.2); // toRadians(degrees)

        public static final int DRIVETRAIN_BACK_LEFT_ANGLE_MOTOR = 28; // CAN
        public static final int DRIVETRAIN_BACK_LEFT_ANGLE_ENCODER = 3; // Analog
        public static final int DRIVETRAIN_BACK_LEFT_DRIVE_MOTOR = 23; // CAN
        public static final double DRIVETRAIN_BACK_LEFT_ENCODER_OFFSET = -Math.toRadians(49.5); // toRadians(degrees)
    }

}
