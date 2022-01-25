// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.classes.Gains;

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
    public static final double kSpeedMultiplier = 1.0;

    public static final class AutoModes {
        public static final int autoNothing = 0;
        public static final int autoMoveForward = 1;
        public static final int autoMoveBackward = 2;

    }

    public static final class InputControllers {
        public static final int kXboxDrive = 0;
        public static final int kXboxOperator = 1;
    }

    /*
     * Constant Values for Limelight based on target and mounting
     */
    public static final class LimeLightValues {
        //TODO: Tune these values
        public static final double steeringP = 0.1;
        public static final double steeringI = 0;
        public static final double steeringD = 0;
        public static final double steeringFeedForward = 0.2;

        public static final double targetHeight = 98.03; // 249 cm
        public static final double targetXPosShoot = 3.83;
        public static final double targetXPosSafeZone = 5;
        public static final double limelightHeight = 21;
        public static final double limelightAngle = 38;
        public static final double kVisionDistanceTolerance = 5;
        public static final double kVisionXTolerance = 5;
    }

    public static final class SwerveDriveGB {
        public static final int kSlotIdx = 0;
        /*
         * Talon SRX/ Victor SPX will supported multiple (cascaded) PID loops. For now
         * we just want the primary one.
         */
        public static final int kPIDLoopIdx = 0;

        /*
         * Set to zero to skip waiting for confirmation, set to nonzero to wait and
         * report to DS if action fails.
         */
        public static final int kTimeoutMs = 30;

        /* Choose so that Talon does not report sensor out of phase */
        public static boolean kSensorPhase = true;

        /*
         * Choose if feedback is non-continuous true: 1023 -> 0 false: 1023 -> 1024
         */
        public static boolean kNonContinuousFeedback = false;

        /* The amount of allowed error in the pid loop */
        public static int kAlloweedError = 7;

        /*
         * Choose based on what direction you want to be positive, this does not affect
         * motor invert.
         */
        public static boolean kMotorInvert = false;

        /*
         * Gains used in Positon Closed Loop, to be adjusted accordingly Gains(kp, ki,
         * kd, kf, izone, peak output);
         */
        public static final Gains kGains = new Gains(20, 0.0, 200, 0.0, 0, 1.0);

        /* Constants for AutoDrive Targeting Mode driven by Vision */
        public static double kVisionDriveSpeedFast = .25;
        public static double kVisionDriveSpeedSlow = .15;
        public static double kInitLineShootingDistance = 180;
        public static double kSafeZoneShootingDistance = 78;
        public static double kVisionDistanceTolerance = 5;
        public static double kVisionXTolerance = .75; // changed from 0.5
        public static double kVisionXToleranceRCW = .5;
        public static double kVisionGyroTolerance = 0.5;

        /* Motor Controllers */
        public static int kFrontLeftSteering = 1;
        public static int kFrontRightSteering = 3;
        public static int kBackLeftSteering = 2;
        public static int kBackRightSteering = 0;
        public static int kFrontLeftDrive = 7;
        public static int kFrontRightDrive = 4;
        public static int kBackLeftDrive = 5;
        public static int kBackRightDrive = 6;
        // #endr
    }

    public static final class SwerveDriveIV {
        public static final int kSlotIdx = 0;
        /*
         * Talon SRX/ Victor SPX will supported multiple (cascaded) PID loops. For now
         * we just want the primary one.
         */
        public static final int kPIDLoopIdx = 0;

        /*
         * Set to zero to skip waiting for confirmation, set to nonzero to wait and
         * report to DS if action fails.
         */
        public static final int kTimeoutMs = 30;

        /* Choose so that Talon does not report sensor out of phase */
        public static boolean kSensorPhase = true;

        /*
         * Choose if feedback is non-continuous true: 1023 -> 0 false: 1023 -> 1024
         */
        public static boolean kNonContinuousFeedback = false;

        /* The amount of allowed error in the pid loop */
        public static int kAlloweedError = 7;

        /*
         * Choose based on what direction you want to be positive, this does not affect
         * motor invert.
         */
        public static boolean kMotorInvert = false;

        /*
         * Gains used in Positon Closed Loop, to be adjusted accordingly Gains(kp, ki,
         * kd, kf, izone, peak output);
         */
        public static final Gains kGains = new Gains(20, 0.0, 200, 0.0, 0, 1.0);

        /* Constants for AutoDrive Targeting Mode driven by Vision */
        public static double kVisionDriveSpeedFast = .25;
        public static double kVisionDriveSpeedSlow = .15;
        public static double kInitLineShootingDistance = 180;
        public static double kSafeZoneShootingDistance = 78;
        public static double kVisionDistanceTolerance = 5;
        public static double kVisionXTolerance = .75; // changed from 0.5
        public static double kVisionXToleranceRCW = .5;
        public static double kVisionGyroTolerance = 0.5;

        /* Motor Controllers */
        public static int kFrontLeftSteering = 14;
        public static int kFrontRightSteering = 17;
        public static int kBackLeftSteering = 11;
        public static int kBackRightSteering = 12;
        public static int kFrontLeftDrive = 15;
        public static int kFrontRightDrive = 16;
        public static int kBackLeftDrive = 10;
        public static int kBackRightDrive = 13;
        // #endr
    }
}