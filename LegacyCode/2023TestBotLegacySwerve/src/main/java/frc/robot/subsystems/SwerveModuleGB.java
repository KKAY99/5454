// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.networktables.GenericEntry;
import frc.robot.Constants;
import frc.robot.classes.MathUtil;

public class SwerveModuleGB {
        private static final double kWheelRadius = 2.0 / 39.37; // 2in to meters
        private static final int kDriveEncoderResolution = 1024;
        private static final int kSteerEncoderResolution = 1024;

        private static final double steeringCoeff = 2.0 * Math.PI / kSteerEncoderResolution;

        private double m_offset;
        private String m_name;

        private ShuffleboardTab tab = Shuffleboard.getTab("Swerve");
        private GenericEntry m_driveOutput;
        private GenericEntry m_steerOutput;
        private GenericEntry m_error;

        // TODO: Adjust this
        // private static final double accelAndVelocAdjust = 0.1;

        private final CANSparkMax m_driveMotor;
        private final TalonSRX m_turningMotor;

        private final RelativeEncoder m_driveEncoder;

        // Gains are for example purposes only - must be determined for your own robot!
        // TODO: Tune these for driving accurately.
        private final PIDController m_drivePIDController = new PIDController(1, 0, 10);

        // Gains are for example purposes only - must be determined for your own robot!
        // TODO: Make sure it steers accurately.
        // private final PIDController m_turningPIDController = new PIDController(2, 0,
        // 100);

        // Gains are for example purposes only - must be determined for your own robot!
        private final SimpleMotorFeedforward m_driveFeedforward = new SimpleMotorFeedforward(1, 3);
        private final Double m_turnFeedforward = 0.00;// .25, 0

        /**
         * Constructs a SwerveModule with a drive motor, turning motor, drive encoder
         * and turning encoder.
         *
         * @param driveMotorChannel      PWM output for the drive motor.
         * @param turningMotorChannel    PWM output for the turning motor.
         * @param driveEncoderChannelA   DIO input for the drive encoder channel A
         * @param driveEncoderChannelB   DIO input for the drive encoder channel B
         * @param turningEncoderChannelA DIO input for the turning encoder channel A
         * @param turningEncoderChannelB DIO input for the turning encoder channel B
         */
        public SwerveModuleGB(int driveMotorID, int turningMotorID, String name, double offset) {
                m_driveMotor = new CANSparkMax(driveMotorID, MotorType.kBrushless);
                m_turningMotor = new TalonSRX(turningMotorID);

                m_name = name;

                m_driveEncoder = m_driveMotor.getEncoder();

                // Set the distance per pulse for the drive encoder. We can simply use the
                // distance traveled for one rotation of the wheel divided by the encoder
                // resolution.
                m_driveEncoder.setPositionConversionFactor(2 * Math.PI * kWheelRadius / kDriveEncoderResolution);

                // Set the distance (in this case, angle) per pulse for the turning encoder.
                // This is the the angle through an entire rotation (2 * pi) divided by the
                // encoder resolution.

                // Limit the PID Controller's input range between -pi and pi and set the input
                // to be continuous.

                // m_turningPIDController.enableContinuousInput(0.0, kSteerEncoderResolution);
                // m_turningPIDController.setTolerance(kSteerEncoderResolution / 2.0);

                m_turningMotor.configSelectedFeedbackSensor(FeedbackDevice.Analog, Constants.SwerveDriveGBConfig.kPIDLoopIdx,
                                Constants.SwerveDriveGBConfig.kTimeoutMs);

                m_turningMotor.configSelectedFeedbackCoefficient(1.0,
                                Constants.SwerveDriveGBConfig.kPIDLoopIdx, Constants.SwerveDriveGBConfig.kTimeoutMs);

                m_turningMotor.configFeedbackNotContinuous(false, Constants.SwerveDriveGBConfig.kTimeoutMs);

                m_turningMotor.setSensorPhase(Constants.SwerveDriveGBConfig.kSensorPhase);

                m_turningMotor.setInverted(Constants.SwerveDriveGBConfig.kMotorInvert);

                m_turningMotor.configNominalOutputForward(0, Constants.SwerveDriveGBConfig.kTimeoutMs);
                m_turningMotor.configNominalOutputReverse(0, Constants.SwerveDriveGBConfig.kTimeoutMs);
                m_turningMotor.configPeakOutputForward(1, Constants.SwerveDriveGBConfig.kTimeoutMs);
                m_turningMotor.configPeakOutputReverse(-1, Constants.SwerveDriveGBConfig.kTimeoutMs);

                m_turningMotor.configAllowableClosedloopError(Constants.SwerveDriveGBConfig.kPIDLoopIdx, 20);
                m_turningMotor.setNeutralMode(NeutralMode.Brake);

                m_driveMotor.setIdleMode(IdleMode.kBrake);

                m_driveMotor.setOpenLoopRampRate(0.5);

                m_driveMotor.setSmartCurrentLimit(40);

                m_driveMotor.burnFlash();

                m_driveOutput = tab.add(name + " drive", 0.0).getEntry();
                m_steerOutput = tab.add(name + " steer", 0.0).getEntry();
                m_error = tab.add(name + "encoder error", 0.0).getEntry();

                m_offset = offset;
        }

        private final double rpmToMetersPerSecond(double RPM) {
                // double wheelRPM = RPM/8.75;
                // double wheelRPS = wheelRPM / 60.0;
                // double MPS = 2.0 * Math.PI * kWheelRadius * wheelRPS;

                return 2.0 * Math.PI * kWheelRadius * ((RPM / 8.75) / 60.0);
        }

        private final double countsToAdjustedRadians(double counts) {
                double radianValue = (counts / 1024.0) * 2.0 * Math.PI;
                double tightenedRadianValue = radianValue % (2 * Math.PI);
                return tightenedRadianValue * steeringCoeff;
        }

        private final double radiansToTicks(double radians) {
                // System.out.println((radians / (2.0 * Math.PI)) * kSteerEncoderResolution);
                return (radians / (2.0 * Math.PI)) * kSteerEncoderResolution;
        }

        /**
         * Returns the current state of the module.
         *
         * @return The current state of the module.
         */
        public SwerveModuleState getState() {
                return new SwerveModuleState(rpmToMetersPerSecond(m_driveEncoder.getVelocity()),
                                new Rotation2d(countsToAdjustedRadians(m_turningMotor.getSelectedSensorPosition())));
        }

        /**
         * Sets the desired state for the module.
         *
         * @param desiredState Desired state with speed and angle.
         */
        public void setDesiredState(SwerveModuleState desiredState) {
                // Optimize the reference state to avoid spinning further than 90 degrees
                SwerveModuleState state = SwerveModuleState.optimize(desiredState,
                                new Rotation2d(countsToAdjustedRadians(m_turningMotor.getSelectedSensorPosition())));

                // Calculate the drive output from the drive PID controller.
                final double driveOutput = m_drivePIDController.calculate(
                                rpmToMetersPerSecond(m_driveEncoder.getVelocity()),
                                state.speedMetersPerSecond);

                final double driveFeedforward = m_driveFeedforward.calculate(state.speedMetersPerSecond);

                // m_error.setDouble(m_turningPIDController.getPositionError());

                // Calculate the turning motor output from the turning PID controller.
                // final double turnOutput = m_turningPIDController.calculate(
                // m_turningMotor.getSelectedSensorPosition(),
                // radiansToTicks(state.angle.getRadians()));

                double driveVoltage = MathUtil.clamp(driveOutput + driveFeedforward, -12.0, 12.0);
                // double steerVoltage = MathUtil.clamp(turnOutput + m_turnFeedforward, -1.0,
                // 1.0);

                m_driveOutput.setDouble(driveVoltage);
                // m_steerOutput.setDouble(steerVoltage);

                // m_driveMotor.setVoltage(driveVoltage);

                final double current = m_turningMotor.getSelectedSensorPosition(Constants.SwerveDriveGBConfig.kSlotIdx);
                final double desired = (int) Math
                                .round(state.angle.getDegrees() * kSteerEncoderResolution / 360.0)
                                + m_offset;

                final double newPosition = (int) MathUtil.minChange(desired, current, kSteerEncoderResolution)
                                + current;

                m_turningMotor.set(ControlMode.Position, newPosition);

                System.out.println(m_name + ": " + m_turningMotor.getSelectedSensorPosition());
        }
}