// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

import frc.robot.Constants;

public class SwerveModule {
    private static final double kWheelRadius = 0.0508;
    private static final int kEncoderResolution = 4096;

    private static final double kModuleMaxAngularVelocity = SwerveSubsystem.kMaxAngularSpeed;
    private static final double kModuleMaxAngularAcceleration = 2 * Math.PI; // radians per second squared

    private final CANSparkMax m_driveMotor;
    private final TalonSRX m_turningMotor;

    private final RelativeEncoder m_driveEncoder;

    // Gains are for example purposes only - must be determined for your own robot!
    // TODO: Tune these for driving accurately.
    private final PIDController m_drivePIDController = new PIDController(1, 0, 10);

    // Gains are for example purposes only - must be determined for your own robot!
    // TODO: Make sure it steers accurately.
    private final ProfiledPIDController m_turningPIDController = new ProfiledPIDController(20, 0, 200,
            new TrapezoidProfile.Constraints(
                    kModuleMaxAngularVelocity, kModuleMaxAngularAcceleration));

    // Gains are for example purposes only - must be determined for your own robot!
    private final SimpleMotorFeedforward m_driveFeedforward = new SimpleMotorFeedforward(1, 3);
    private final SimpleMotorFeedforward m_turnFeedforward = new SimpleMotorFeedforward(1, 0.5);

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
    public SwerveModule(
            int driveMotorID,
            int turningMotorID) {
        m_driveMotor = new CANSparkMax(driveMotorID, MotorType.kBrushless);
        m_turningMotor = new TalonSRX(turningMotorID);

        m_driveEncoder = m_driveMotor.getEncoder();
        // m_turningEncoder = m_turningMotor.

        // // Set the distance per pulse for the drive encoder. We can simply use the
        // // distance traveled for one rotation of the wheel divided by the encoder
        // // resolution.
        m_driveEncoder.setPositionConversionFactor(2 * Math.PI * kWheelRadius / kEncoderResolution);

        // // Set the distance (in this case, angle) per pulse for the turning encoder.
        // // This is the the angle through an entire rotation (2 * pi) divided by the
        // // encoder resolution.
        m_turningMotor.configSelectedFeedbackCoefficient(2 * Math.PI / kEncoderResolution,
                Constants.SwerveDriveGB.kPIDLoopIdx, Constants.SwerveDriveGB.kTimeoutMs);
        // m_turningEncoder.setDistancePerPulse(2 * Math.PI / kEncoderResolution);

        // Limit the PID Controller's input range between -pi and pi and set the input
        // to be continuous.

        m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);

        m_turningMotor.configSelectedFeedbackSensor(FeedbackDevice.Analog, Constants.SwerveDriveGB.kPIDLoopIdx,
                Constants.SwerveDriveGB.kTimeoutMs);

        m_turningMotor.setSensorPhase(Constants.SwerveDriveGB.kSensorPhase);

        m_turningMotor.setInverted(Constants.SwerveDriveGB.kMotorInvert);

        m_turningMotor.configNominalOutputForward(0, Constants.SwerveDriveGB.kTimeoutMs);
        m_turningMotor.configNominalOutputReverse(0, Constants.SwerveDriveGB.kTimeoutMs);
        m_turningMotor.configPeakOutputForward(1, Constants.SwerveDriveGB.kTimeoutMs);
        m_turningMotor.configPeakOutputReverse(-1, Constants.SwerveDriveGB.kTimeoutMs);

        m_driveMotor.setIdleMode(IdleMode.kBrake);

        m_driveMotor.setOpenLoopRampRate(0.5);

        m_driveMotor.setSmartCurrentLimit(40);

        m_driveMotor.burnFlash();
    }

    /**
     * Returns the current state of the module.
     *
     * @return The current state of the module.
     */
    public SwerveModuleState getState() {
        return new SwerveModuleState(m_driveEncoder.getVelocity(),
                new Rotation2d(m_turningMotor.getSelectedSensorPosition()));
    }

    /**
     * Sets the desired state for the module.
     *
     * @param desiredState Desired state with speed and angle.
     */
    public void setDesiredState(SwerveModuleState desiredState) {
        // Optimize the reference state to avoid spinning further than 90 degrees
        SwerveModuleState state = SwerveModuleState.optimize(desiredState,
                new Rotation2d(m_turningMotor.getSelectedSensorPosition()));

        // Calculate the drive output from the drive PID controller.
        final double driveOutput = m_drivePIDController.calculate(m_driveEncoder.getVelocity(),
                state.speedMetersPerSecond);

        final double driveFeedforward = m_driveFeedforward.calculate(state.speedMetersPerSecond);

        // Calculate the turning motor output from the turning PID controller.
        final double turnOutput = m_turningPIDController.calculate(m_turningMotor.getSelectedSensorPosition(),
                state.angle.getRadians());

        final double turnFeedforward = m_turnFeedforward.calculate(m_turningPIDController.getSetpoint().velocity);

        m_driveMotor.setVoltage(driveOutput + driveFeedforward);
        m_turningMotor.set(ControlMode.PercentOutput, turnOutput + turnFeedforward);
    }
}