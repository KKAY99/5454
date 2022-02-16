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
    public static final int ConveyorPort=33;
    public static final int IntakePort=13;
    public static final int TurretPort=14;
    public static final int FeederPort=15;
    public static final double intakeSpeed=-.45;
    public static final double FeederSpeed=.5;
    public static final double conveyorUpSpeed=0.9;
    public static final double conveyorDownSpeed=-0.5;
  
    public static final class ButtonConstants{
       public static final int ManualShoot=1;
       public static final int AimandShoot=2;
       public static final int ConveyerIn=6;
       public static final int ConveyerOut=5;
       public static final int IntakeIn=3;
       public static final int IntakeOut=4;
       public static final int FeederUp=8;
    }

    public static final class AutoModes {
        public static final String autoMode0="Do Nothing";
        public static final String autoMode1="Move Foward";
        public static final String autoMode2="Shoot /  Move";
        public static final String autoMode3="Outake / Move Backwars";
        public static final String autoMode4="Shoot / Move Backwards";
        public static final String autoMode5="Shoot Move Grab";
        public static final String autoMode6="Shoot Move Grab Shot 1";
        public static final String autoMode7="Shoot Move Grab Move Left Grab Shoot 2";
        public static final String autoMode8="Shoot Move Grab Move Right Grab Shoot 2";
        public static final String autoMode9="Move Grab Track Right / Shoot";
        public static final String autoMode10="Move Grab Track Left / Shoot";
        public static final int defaultAutoMode=1;

        public static final int autoNothing = 0;
        public static final int autoMoveForward = 1;
        public static final int autoMoveShoot = 2;
        public static final int autoMoveBackwardsOutake = 3;
        public static final int autoMoveBackwardsShot = 4;
        public static final int autoMoveShootMoveGrab = 5;
        public static final int autoMoveShootMoveGrabShot1 = 6;
        public static final int autoMoveShotMoveGrabMoveLeftGrabShot2 = 7;
        public static final int autoMoveShotMoveGrabMoveRightGrabShot2 = 8;
        public static final int autoMoveGrabTrackRightShoot = 9;
        public static final int autoMoveGrabTrackLeftShoot = 10;
        
        public static final String delayMode0="0 Seconds";
        public static final String delayMode1="3 Seconds";
        public static final String delayMode2="5 Seconds";
        public static final String delayMode3="8 Seconds";
        public static final int delayValMode0=0;
        public static final int delayValMode1=3;
        public static final int delayValMode2=5;
        public static final int delayValMode3=8;
        public static final int defaultDelayMode=0;

        public static final double LeaveTarmacDistance=1.0;
        public static final double ball2Distance=.5;
            
        
        public static final double DistanceToBall1=7;
        public static final double MoveSpeed=.30;
        public static final double AutoShotTopSpeed=800;
        public static final double AutoShotBottomSpeed=800;
        public static final double AutoMinVelocity=800;
    }
    public static final class zAutomation{
        public static final double shooterSpinUp=2;
        public static final double conveyorfeedBall=2;
        public static final double intakeTime=.1;
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
        public static final double steeringP = 0.035;
        public static final double steeringI = 0;
        public static final double steeringD = 0.0055;
        public static final double steeringFeedForward = 0.0;

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
    public static final class SwerveDriveNEO {
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
        public static final Gains kGains = new Gains(5e-5,1e-6,0,0.000156,0,1.0);
        public static final int kMaxOutput=1;
        public static final int kMinOutput=-1;
        public static final int maxVel=5000;
        public static final int maxACC=2500;
        public static int smartMotionSlot=0;
        public static double allowedErr=1; //GUESSSSSSS

        public static double minVel = 0; // rpm
         
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
        public static int kFrontLeftSteering = 25;
        public static int kFrontRightSteering = 26;
        public static int kBackLeftSteering = 28;
        public static int kBackRightSteering = 27;
        public static int kFrontLeftDrive = 20;
        public static int kFrontRightDrive = 21;
        public static int kBackLeftDrive = 22;
        public static int kBackRightDrive = 23;
        // #endr
        
        public static int kFrontLeftSensor=0;
        public static int kFrontRightSensor=1;
        public static int kBackLeftSensor=2;
        public static int kBackRightSensor=3;
        
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
        public static final Gains kGains = new Gains(0.5, 0.0, 0.0001, 0.0, 0, 1.0);

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
    
public static class RobotMap {
    public static final int DRIVETRAIN_FRONT_LEFT_MODULE_ANGLE_ENCODER = 0;
    public static final int DRIVETRAIN_FRONT_LEFT_MODULE_ANGLE_MOTOR = 4;
    public static final double DRIVETRAIN_FRONT_LEFT_MODULE_ANGLE_OFFSET = -Math.toRadians(154.3);
    public static final int DRIVETRAIN_FRONT_LEFT_MODULE_DRIVE_MOTOR = 10;

    public static final int DRIVETRAIN_FRONT_RIGHT_MODULE_ANGLE_ENCODER = 3;
    public static final int DRIVETRAIN_FRONT_RIGHT_MODULE_ANGLE_MOTOR = 7;
    public static final double DRIVETRAIN_FRONT_RIGHT_MODULE_ANGLE_OFFSET = -Math.toRadians(329.0);
    public static final int DRIVETRAIN_FRONT_RIGHT_MODULE_DRIVE_MOTOR = 6;

    public static final int DRIVETRAIN_BACK_LEFT_MODULE_ANGLE_ENCODER = 1;
    public static final int DRIVETRAIN_BACK_LEFT_MODULE_ANGLE_MOTOR = 5;
    public static final double DRIVETRAIN_BACK_LEFT_MODULE_ANGLE_OFFSET = -Math.toRadians(218.1);
    public static final int DRIVETRAIN_BACK_LEFT_MODULE_DRIVE_MOTOR = 9;

    public static final int DRIVETRAIN_BACK_RIGHT_MODULE_ANGLE_ENCODER = 2;
    public static final int DRIVETRAIN_BACK_RIGHT_MODULE_ANGLE_MOTOR = 6;
    public static final double DRIVETRAIN_BACK_RIGHT_MODULE_ANGLE_OFFSET = -Math.toRadians(268.9);
    public static final int DRIVETRAIN_BACK_RIGHT_MODULE_DRIVE_MOTOR = 7;
    }

}