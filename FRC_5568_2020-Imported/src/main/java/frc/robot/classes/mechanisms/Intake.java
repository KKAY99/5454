/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.classes.mechanisms;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.Solenoid;
import frc.robot.classes.constants.*;

/**
 * This class is used to run swerve drive.
 */
public class Intake {
    // Create TalonSRX Speed Controllers
    private VictorSPX m_IntakeMotor;

    private Solenoid m_IntakePneumatic;

    public double maxIntakeSpeed = 1.0;
    public double maxOuttakeSpeed = .75;

    /**
     * Configures the drivebase with Steering TalonSRXs and Speed Controller Drives
     * 
     * @param intakeMotor The VictorSPX for the intake motor
     */

    public Intake(VictorSPX intakeMotor, Solenoid intakePneum) {

        m_IntakePneumatic = intakePneum;
        m_IntakeMotor = intakeMotor;

        m_IntakeMotor.setInverted(false);

        /* Config the peak and nominal outputs, 12V means full */
        // Top
        m_IntakeMotor.configNominalOutputForward(0, Constants.kTimeoutMs);
        m_IntakeMotor.configNominalOutputReverse(0, Constants.kTimeoutMs);
        m_IntakeMotor.configPeakOutputForward(1, Constants.kTimeoutMs);
        m_IntakeMotor.configPeakOutputReverse(-1, Constants.kTimeoutMs);

    }

    /**
     * Runs the intake at a speed that will max out at the constant max intake speed
     * 
     * @param speed The intake speed
     */
    public void intake(double speed) {
        speed = Math.abs(speed);
        if (speed > maxIntakeSpeed) {
            speed = maxIntakeSpeed;
        }
        m_IntakeMotor.set(ControlMode.PercentOutput, speed);
    }

    /**
     * Runs the intake at a backwards speed that will max out at the constant max
     * outtake speed
     * 
     * @param speed The outake speed
     */
    public void outtake(double speed) {
        speed = Math.abs(speed);
        if (speed > maxOuttakeSpeed) {
            speed = maxOuttakeSpeed;
        }
        speed *= -1;
        m_IntakeMotor.set(ControlMode.PercentOutput, speed);
    }

    public void intakeKill() {
        m_IntakeMotor.set(ControlMode.PercentOutput, 0);
    }

    public void raise() {
        m_IntakePneumatic.set(false);
    }

    public void lower() {
        m_IntakePneumatic.set(true);
    }

}
