// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTableEntry.*;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

/** Represents a swerve drive style drivetrain. */
public class SwerveSubsystem extends SubsystemBase {
        public static final double kMaxSpeed = 3.0; // 3 meters per second
        public static final double kMaxAngularSpeed = Math.PI; // 1/2 rotation per second

        // public static final int frontLeftOffset = -513;
        // public static final int frontRightOffset = -253;
        // public static final int backLeftOffset = -640;
        // public static final int backRightOffset = -914;

        public static final int frontLeftOffset = 0;
        public static final int frontRightOffset = 0;
    
        public static final int backLeftOffset = -10;
        public static final int backRightOffset = 256;

        // TODO: Measure this (it's in meters);
        // Note the signs
        // FL: + +
        // FR: + -
        // BL: - +
        // BR: - -
        private final Translation2d m_frontLeftLocation = new Translation2d(0.381, 0.381);
        private final Translation2d m_frontRightLocation = new Translation2d(0.381, -0.381);
        private final Translation2d m_backLeftLocation = new Translation2d(-0.381, 0.381);
        private final Translation2d m_backRightLocation = new Translation2d(-0.381, -0.381);

        private GenericEntry frontLeftSpeed = Shuffleboard.getTab("Swerve").add("FL_S", 0.0).getEntry();
        private GenericEntry frontRightSpeed = Shuffleboard.getTab("Swerve").add("FR_S", 0.0).getEntry();
        private GenericEntry backLeftSpeed = Shuffleboard.getTab("Swerve").add("BL_S", 0.0).getEntry();
        private GenericEntry backRightSpeed = Shuffleboard.getTab("Swerve").add("BR_S", 0.0).getEntry();

        private GenericEntry frontLeftAngle = Shuffleboard.getTab("Swerve").add("FL_A", 0.0).getEntry();
        private GenericEntry frontRightAngle = Shuffleboard.getTab("Swerve").add("FR_A", 0.0).getEntry();
        private GenericEntry backLeftAngle = Shuffleboard.getTab("Swerve").add("BL_A", 0.0).getEntry();
        private GenericEntry backRightAngle = Shuffleboard.getTab("Swerve").add("BR_A", 0.0).getEntry();

        private final SwerveModuleGB m_frontLeft = new SwerveModuleGB(Constants.SwerveDriveGB.kFrontLeftDrive,
                        Constants.SwerveDriveGB.kFrontLeftSteering, "Front Left", frontLeftOffset);
        private final SwerveModuleGB m_frontRight = new SwerveModuleGB(Constants.SwerveDriveGB.kFrontRightDrive,
                        Constants.SwerveDriveGB.kFrontRightSteering, "Front Right", frontRightOffset);
        private final SwerveModuleGB m_backLeft = new SwerveModuleGB(Constants.SwerveDriveGB.kBackLeftDrive,
                        Constants.SwerveDriveGB.kBackLeftSteering, "Back Left", backLeftOffset);
        private final SwerveModuleGB m_backRight = new SwerveModuleGB(Constants.SwerveDriveGB.kBackRightDrive,
                        Constants.SwerveDriveGB.kBackRightSteering, "Back Right", backRightOffset);

        private final AHRS m_gyro = new AHRS(SPI.Port.kMXP);

        private final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
                        m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);
        //TODO : Fix for 2023
       // private final SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(m_kinematics, m_gyro.getRotation2d());
        public SwerveSubsystem() {
                System.out.println("***Swerve instanciated***");
                m_gyro.reset();
        }

        /**
         * Method to drive the robot using joystick info.
         *
         * @param xSpeed Speed of the robot in the x direction (forward).
         * @param ySpeed Speed of the robot in the y direction (sideways).
         * @param rot    Angular rate of the robot.
         *               field.
         */
        @SuppressWarnings("ParameterName")
        public void drive(double xSpeed, double ySpeed, double rot) {
                drive(xSpeed, ySpeed, rot, false);
        }

        /**
         * Method to drive the robot using joystick info.
         *
         * @param xSpeed        Speed of the robot in the x direction (forward).
         * @param ySpeed        Speed of the robot in the y direction (sideways).
         * @param rot           Angular rate of the robot.
         * @param fieldRelative Whether the provided x and y speeds are relative to the
         *                      field.
         */
        @SuppressWarnings("ParameterName")
        public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
                var swerveModuleStates = m_kinematics.toSwerveModuleStates(
                                fieldRelative
                                                ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot,
                                                                m_gyro.getRotation2d())
                                                : new ChassisSpeeds(xSpeed, ySpeed, rot));
                SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, kMaxSpeed);
                m_frontLeft.setDesiredState(swerveModuleStates[0]);
                m_frontRight.setDesiredState(swerveModuleStates[1]);
                m_backLeft.setDesiredState(swerveModuleStates[2]);
                m_backRight.setDesiredState(swerveModuleStates[3]);

                frontLeftSpeed.setDouble(swerveModuleStates[0].speedMetersPerSecond);
                frontRightSpeed.setDouble(swerveModuleStates[1].speedMetersPerSecond);
                backLeftSpeed.setDouble(swerveModuleStates[2].speedMetersPerSecond);
                backRightSpeed.setDouble(swerveModuleStates[3].speedMetersPerSecond);

                frontLeftAngle.setDouble(swerveModuleStates[0].angle.getDegrees());
                frontRightAngle.setDouble(swerveModuleStates[1].angle.getDegrees());
                backLeftAngle.setDouble(swerveModuleStates[2].angle.getDegrees());
                backRightAngle.setDouble(swerveModuleStates[3].angle.getDegrees());

                updateOdometry();
        }

        /** Updates the field relative position of the robot. */
        public void updateOdometry() {
        //TODO: Disabled for 2023 beta
        /*         m_odometry.update(
                                m_gyro.getRotation2d(),
                                m_frontLeft.getState(),
                                m_frontRight.getState(),
                                m_backLeft.getState(),
                                m_backRight.getState());
        */}

        public void move(double direction, double speed, double distance, boolean stopAtFalse) {
                // TODO - Nothing Currently
        }
}