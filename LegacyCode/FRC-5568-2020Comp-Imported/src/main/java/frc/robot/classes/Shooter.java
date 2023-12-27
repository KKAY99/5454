/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.classes;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;

/**
 * This class is used to run the shooter.
 */
public class Shooter {
    // Create TalonSRX Speed Controllers
    private CANSparkMax m_TopMotor;
    private CANSparkMax m_BottomMotor;

    private Solenoid m_Long;
    private Solenoid m_Short;

    // Create Variables
    private int ticksPerRevolution = 8192;

    private CANPIDController m_TopPidController;
    private CANEncoder m_TopEncoder;
    public static final double kP = 0.0;
    public static final double kI = 0.0;
    public static final double kD = 0.0;
    public static final double kIz = 0.0;
    public static final double kFF = 0.0;// 0.0
    public static final double kMaxOutput = 1.0; // 1.0
    public static final double kMinOutput = -1.0; // -1.0
    public static final double maxRPM = Double.MAX_VALUE; // large number

    // power values are based off voltage run
    // distance values are in feet
    // encoder values are in RPM
    public static double[] powerValues = { 0.0, 0.3, 0.6, 1.0 };
    public static int[] distanceValues = { 0, 5, 10, 20 };
    public static int[] encoderValues = { 0, 0, 0, 0 };

    /**
     * Configures the drivebase with Steering TalonSRXs and Speed Controller Drives
     * 
     * @param topMotor    The SparkMax for the top motor
     * @param bottomMotor The SparkMax for the bottom motor
     */

    public Shooter(CANSparkMax topMotor, CANSparkMax bottomMotor, Solenoid longPneumatic, Solenoid shortPneumatic) {

        m_TopMotor = topMotor;
        m_BottomMotor = bottomMotor;

        m_Long = longPneumatic;
        m_Short = shortPneumatic;

        /**
         * The RestoreFactoryDefaults method can be used to reset the configuration
         * parameters in the SPARK MAX to their factory default state. If no argument is
         * passed, these parameters will not persist between power cycles
         */
        // m_TopMotor.restoreFactoryDefaults();
        // m_BottomMotor.restoreFactoryDefaults();

        /**
         * In order to use PID functionality for a controller, a CANPIDController object
         * is constructed by calling the getPIDController() method on an existing
         * CANSparkMax object
         */
        // m_TopPidController = m_TopMotor.getPIDController();

        // Encoder object created to display position values
        // m_TopEncoder = m_TopMotor.getEncoder();

        // // set PID coefficients
        // m_TopPidController.setP(kP);
        // m_TopPidController.setI(kI);
        // m_TopPidController.setD(kD);
        // m_TopPidController.setIZone(kIz);
        // m_TopPidController.setFF(kFF);
        // m_TopPidController.setOutputRange(kMinOutput, kMaxOutput);
    }

    public void PIDLoop() {

        /**
         * PIDController objects are commanded to a set point using the SetReference()
         * method.
         * 
         * The first parameter is the value of the set point, whose units vary depending
         * on the control type set in the second parameter.
         * 
         * The second parameter is the control type can be set to one of four
         * parameters: com.revrobotics.ControlType.kDutyCycle
         * com.revrobotics.ControlType.kPosition com.revrobotics.ControlType.kVelocity
         * com.revrobotics.ControlType.kVoltage
         */
        double setPoint = 0;
        m_TopPidController.setReference(setPoint, ControlType.kVelocity);

        SmartDashboard.putNumber("SetPoint", setPoint);
        SmartDashboard.putNumber("ProcessVariable", m_TopEncoder.getVelocity());
    }

    public double getEncoderSpeed() {
        return m_TopEncoder.getVelocity();
    }

    /**
     * Runs the shooter at a set velocity for the top and bottom wheels
     * 
     * * @param velocity The Velocity value.
     */
    public void velocityShoot(double velocity) {
        m_TopPidController.setReference(velocity, ControlType.kVelocity);
    }

    public void distanceShoot(double distance) {
        voltageRun(getPower(distance));
    }

    public static double getPower(double distance) {
        distance = Math.max(distance, 0);
        for (int i = 0; i < distanceValues.length; i++) {
            if (distanceValues[i] == distance) {
                return powerValues[i];
            } else if (distanceValues[i] > distance) {
                return getEquation(distance, distanceValues[i], powerValues[i], distanceValues[i - 1],
                        powerValues[i - 1]);
            }
        }
        return 1;
    }

    private static double getEquation(double value, double xOne, double yOne, double xTwo, double yTwo) {
        double slope = (yTwo - yOne) / (xTwo - xOne);
        return (slope * (value - xOne)) + yOne;
    }

    public void slowRun(double power) {
        m_TopMotor.set(-power);
        m_BottomMotor.set(-power);
    }

    public void voltageRun(double power) {
        power = Math.abs(power) * 11.0;
        //KK 2/7  no longer needs to be reversed
        // power *= -1;
        m_TopMotor.setVoltage(power);
        m_BottomMotor.setVoltage(power);
    }

    public double topDistanceToVelocity(double distance) {
        return distance;
    }

    public double bottomDistanceToVelocity(double distance) {
        return distance;
    }

    public void shooterKill() {
        m_TopMotor.set(0);
        m_BottomMotor.set(0);
    }

    public int rpmToNativeUnits(double input) {
        int output = ((int) input * ticksPerRevolution) / 600;
        return output;
    }

    public void longPneumatic(boolean isUp) {
        m_Long.set(isUp);
    }

    public void shortPneumatic(boolean isUp) {
        m_Short.set(isUp);
    }

}
