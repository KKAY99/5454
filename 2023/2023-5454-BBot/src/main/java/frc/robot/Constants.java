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

    public static final class ChargedUp {
        public static final double targetHeightAprilTag=0.4572;     // middle of April Tag in Distance
        public static final double targetHeighMLowTape=24.125;  // middle of low tape in inches
        public static final double targetHeightHighTape=43.875;
        public static final int apriltag=0;
        public static final int middleTape=1;
        public static final int BottomTapePipeline=0;
        public static final double swerveDeadband=0.05;
    }

    public static final class Pneumatics {
        public static final int CompressorID=0; 
        public static final int LatchPort=5;
        public static final int clawSolenoid1ID=1;
        public static final int clawSolenoid2ID=0;
        public static final int extensionSolenoidID=2;
        public static final int holdSolenoidID=4;
    }

    public static final class RotateArm{
        public static final int rotateMotorPort=33;
        public static final int absEncoderPort=1;
        public static final double FWDSoftLimitABS=0.07;
        public static final double BWDSoftLimitABS=0.387;
    }

    public static final class Rotate{
        public static final double rotateAutoOutStage1Speed=-0.50;
        public static final double rotateAutoOutStage2Speed=-0.13; 
        public static final double rotateAutoInSpeed=0.8;     
        public static final double angleMiddleConeABS=0;
        public static final double angleMiddleCubeABS=0;
        public static final double anglePlayerStageABS=0;
        public static final double angleSlideABS=0;
        public static final double angleIntakePos=0.02;
        public static final double ABSHomePos = 0.536;
        public static final double homeTimeFailsafe=5;
        public static final double homeSpeedForward=0.06;
        public static final double homeSpeedBackward=-0.06;
        public static final double encoderLowScorePos=0;
        public static final double posMiddleConeFullLiftStage1=0;
        public static final double posCubeOutofLimelight=0;

        public static final double KP = 0.1;
        public static final double KI = 1e-4;
        public static final double KD = 1;
        public static final double KIZ = 0;
        public static final double KFF = 0;
        public static final double maxOutput = 1;
        public static final double minOutPut = -1;

        public static final double autoRotateStarting=0.997;
        public static final double autoRotatePlayerStation=0.897;
        public static final double autoRotateScore=0.548;
    }

    public enum SETPOSPOSITIONS{
        STARTING,PLAYERSTATION,SCORE
    }

    public static final class PIDSteering{
        public static final double rightKP=-0.09;
        public static final double leftKP=0.09;
        public static final double rightKI=-0;  
        public static final double leftKI=0;
        public static final double rightKD=-0;
        public static final double leftKD=0;
        public static final double forwardKP= -0.18;
        public static final double backwardKP = 0.18;
        public static final double forwardKI = -0;
        public static final double backwardKI = 0;
        public static final double forwardKD = -0;
        public static final double backwardKD = 0;

    }

    public static final class ButtonConstants{
        public static final double RotateDeadBand = 0.05;
    }

    public static final class LimitSwitches{
        public static final int ClimberBottom=9;
        public static final int TurretLeft=0;
        public static final int TurretRight=1;
        public static final int ClimberTop=2;
        public static final double TurretRightEncoder=-0.5;
        public static final double TurretLeftEncoder=-7.93;
    }

    public static final class AutoModes {
        public static final String autoMode0="Do Nothing";
        public static final String autoMode1="Move Foward/Back";
        public static final String autoMode2="Score-Move Back";

        public static final int autoNothing=0;
        public static final int autoMoveForward=1;
        public static final int autoScoreMoveBWD=2;

        public static final double MoveSpeed =0.5;
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

        public static final double targetHeight = 14.875; //  cm
        public static final double targetXPosShoot = -1.5;
        public static final double targetXPosSafeZone = 5;
        public static final double targetXPosRange=50;
        public static final double limelightHeight = 27.5; //37
        public static final double limelightAngle = 0; //40
        public static final double kVisionDistanceTolerance = 5;
        public static final double kVisionXTolerance = 1.5;
        //public static final double kVisionXOffset=3.7; // was 1.7
        public static final double kVisionXOffset=4;
        public static final double kVisionXMaxDistanceOffset=4.31; // was 1.7
        public static final double kVisionXMinDistanceOffset=0.91; // was 1.7
    }

    public static final double shooterPrimedSpeed=000; //was 500 changed at stemposium
    public static final double shooterSweetSpotLow=70; //was 60
    public static final double shooterSweetSpotHigh=90;
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
        public static int kFrontLeftSteering = 30;
        public static int kFrontRightSteering = 1;
        public static int kBackLeftSteering = 4;
        public static int kBackRightSteering = 2;
        public static int kFrontLeftDrive = 32;
        public static int kFrontRightDrive = 6;
        public static int kBackLeftDrive = 7;
        public static int kBackRightDrive = 4;
        // #endr

        public static final double driveDeadband=0.05;
    }

    public static enum TargetHeight
    {
                    TOPCONE,MIDDLECONE,BOTTOMCONE,TOPCUBE,MIDDLECUBE,BOTTOMCUBE,PLAYERSTATION,SLIDE;	
    }
    
    public class LEDS {
        public static final int PORT = 0;
        public static final int COUNT = 215;
        public static final int FLASH_DELAY=5;
        ;

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
        public static double kVisionDistanceTolerance = 10;
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