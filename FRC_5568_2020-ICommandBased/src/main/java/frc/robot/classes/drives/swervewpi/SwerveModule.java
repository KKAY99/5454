/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019-2020 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.classes.drives.swervewpi;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;

import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;

import frc.robot.classes.constants.*;
import frc.robot.classes.utilfuncs.MathUtil;

public class SwerveModule {
    private static final double kWheelRadius = 0.0508; // In Meters
    private static final double kWheelCircumference = 2 * Math.PI * kWheelRadius;

    // Number of rotations the motor makes for the wheel to spin once
    private static final double kGearRatio = 8.75;
    private int m_moduleOffset = 0;

    // Based on arbritary calculation
    // Value you're provided in numerator, value you're converting to in denominator
    // Useful to know for future conversions: ex: ticks to degrees
    private static final double TICKS_TO_RADIANS = 1024.0 / (2 * Math.PI);

    private final CANSparkMax m_driveMotor;
    private final TalonSRX m_turningMotor;

    private final CANEncoder m_driveEncoder;

    // Class level declaration so it can have a public getter
    private int desiredEncoderPositon = 0;

    // Because we only use this for velocity control it doesn't need anything but a
    // proportional value
    private final PIDController m_drivePIDController = new PIDController(1, 0, 0);

    // Gains are for example purposes only - must be determined for your own robot!
    private final SimpleMotorFeedforward m_driveFeedforward = new SimpleMotorFeedforward(1, 3);

    /**
     * Constructs a SwerveModule.
     *
     * @param driveMotorChannel   ID for the drive motor.
     * 
     * @param turningMotorChannel ID for the turning motor.
     */
    public SwerveModule(int driveMotorChannel, int turningMotorChannel, int moduleOffset) {
    
        m_driveMotor = new CANSparkMax(driveMotorChannel, MotorType.kBrushless);
        m_turningMotor = new TalonSRX(turningMotorChannel);
        m_moduleOffset = moduleOffset;

        //m_driveEncoder = new CANEncoder(m_driveMotor);
        m_driveEncoder = m_driveMotor.getEncoder();
    
        

        // Set the multiplier so when we get position it's in the correct units, in this
        // case meters
        m_driveEncoder.setPositionConversionFactor(kWheelCircumference / kGearRatio);

        // Set the multiplier so when we get velocity it's in the correct units, in this
        // case meters per second
        m_driveEncoder.setVelocityConversionFactor((kWheelCircumference / kGearRatio) / 60.0);

        m_driveMotor.setMotorType(MotorType.kBrushless);
        m_driveMotor.setIdleMode(IdleMode.kBrake);
        m_driveMotor.setOpenLoopRampRate(0.5);
        m_driveMotor.setSmartCurrentLimit(40);
        
        m_driveMotor.burnFlash();

        m_turningMotor.configSelectedFeedbackSensor(FeedbackDevice.Analog, Constants.kPIDLoopIdx, Constants.kTimeoutMs);

        // Tell the talon to not report if the sensor is out of Phase
        m_turningMotor.setSensorPhase(Constants.kSensorPhase);

        // Set based on what direction you want forward/positive to be. This does not
        // affect sensor phase.
        m_turningMotor.setInverted(Constants.kMotorInvert);

        /* Config the peak and nominal outputs, 12V means full */
        // Front Left
        m_turningMotor.configNominalOutputForward(0, Constants.kTimeoutMs);
        m_turningMotor.configNominalOutputReverse(0, Constants.kTimeoutMs);
        m_turningMotor.configPeakOutputForward(1, Constants.kTimeoutMs);
        m_turningMotor.configPeakOutputReverse(-1, Constants.kTimeoutMs);

        // Configure the amount of allowable error in the loop
        m_turningMotor.configAllowableClosedloopError(Constants.kAlloweedError, Constants.kPIDLoopIdx,
                Constants.kTimeoutMs);

        // Configure the overloop behavior of the encoders
        m_turningMotor.configFeedbackNotContinuous(Constants.kNonContinuousFeedback, Constants.kTimeoutMs);

        /* Config Position Closed Loop gains in slot0, typically kF stays zero. */
        // Front Left
        m_turningMotor.config_kF(Constants.kPIDLoopIdx, Constants.kGains.kF, Constants.kTimeoutMs);
        m_turningMotor.config_kP(Constants.kPIDLoopIdx, Constants.kGains.kP, Constants.kTimeoutMs);
        m_turningMotor.config_kI(Constants.kPIDLoopIdx, Constants.kGains.kI, Constants.kTimeoutMs);
        m_turningMotor.config_kD(Constants.kPIDLoopIdx, Constants.kGains.kD, Constants.kTimeoutMs);
        System.out.println("Swerve Module constructor COMPLETED");
    }

    /**
     * Returns the current state of the module.
     *
     * @return The current state of the module.
     */
    public SwerveModuleState getState() {
        return new SwerveModuleState(m_driveEncoder.getVelocity(),
                new Rotation2d(m_turningMotor.getSelectedSensorPosition() * TICKS_TO_RADIANS));
    }

    /**
     * Sets the desired state for the module.
     *
     * @param state Desired state with speed and angle.
     */
    public void setDesiredState(SwerveModuleState state) {
        // Calculate the drive output from the drive PID controller.
        final double driveOutput = m_drivePIDController.calculate(m_driveEncoder.getVelocity(),
                state.speedMetersPerSecond);

        final double driveFeedforward = m_driveFeedforward.calculate(state.speedMetersPerSecond);
        //System.out.println("Voltage-" + (driveOutput + driveFeedforward));
        m_driveMotor.setVoltage(driveOutput + driveFeedforward);
        // Calculate the turning motor output from the turning PID controller.
        // Angle state.angle.getRadians()

        final int ticksPerRotation = 1024; // in encoder counts
        final int current =(int)  m_turningMotor.getSelectedSensorPosition(Constants.kSlotIdx);
        desiredEncoderPositon = (int) Math.round(state.angle.getDegrees() * ticksPerRotation / 360.0) + m_moduleOffset;

        final int newPosition = (int) MathUtil.minChange(desiredEncoderPositon, current, ticksPerRotation) + current;
        System.out.println("Turning to " + state.angle.getDegrees() + " from " + current + " to "  + newPosition );
        m_turningMotor.set(ControlMode.Position, newPosition);
    }

    /**
     * Get's the current steering encoder value
     *
     */
    public int getSteerEncoderActual() {
        return (int) m_turningMotor.getSelectedSensorPosition(Constants.kSlotIdx);
    }

    /**
     * Get's the taget steering encoder value
     */
    public int getSteerEncoderTarget() {
        return (int) m_turningMotor.getSelectedSensorPosition(Constants.kSlotIdx);
    }

    public int getOffset(){
        return m_moduleOffset;
    }

    public void spinWheel(double speed){
      //  m_driveMotor.set(speed);
          m_driveMotor.setVoltage(30.0003119999998944);
    }
    

}