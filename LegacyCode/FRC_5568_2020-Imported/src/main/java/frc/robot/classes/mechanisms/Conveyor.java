/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.classes.mechanisms;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

/**
 * This class is used to run swerve drive.
 */
public class Conveyor {
    // Create TalonSRX Speed Controllers
    private TalonSRX m_ConveyorControllerTop;
    private TalonSRX m_ConveyorControllerBottom;

    // Create Variables
    public static final double loadSpeed = 0.75;
    public static final double feedSpeed = 1.0;

    /**
     * Configures the Conveyor
     * 
     * @param ConveyorMotor The TalonSRX for the conveyor motors
     */

    public Conveyor(TalonSRX ConveyorMotorTop, TalonSRX ConveyorMotorBottom) {

        m_ConveyorControllerTop = ConveyorMotorTop;
        m_ConveyorControllerBottom = ConveyorMotorBottom;
        m_ConveyorControllerTop.configFactoryDefault();
        m_ConveyorControllerBottom.configFactoryDefault();
        m_ConveyorControllerTop.setInverted(true);
        m_ConveyorControllerBottom.setInverted(false);
    }

    /**
     * Runs the conveyor towards the shooter at feed speed
     */
    public void feedShooter() {
        m_ConveyorControllerTop.set(ControlMode.PercentOutput, feedSpeed);
        m_ConveyorControllerBottom.set(ControlMode.PercentOutput, feedSpeed);
    }

    public void loadConveyor() {
        loadConveyor(loadSpeed);
    }

    public void loadConveyor(double power) {
        m_ConveyorControllerTop.set(ControlMode.PercentOutput, power);
        m_ConveyorControllerBottom.set(ControlMode.PercentOutput, power);
    }

    public void runReverse() {
        m_ConveyorControllerTop.set(ControlMode.PercentOutput, -loadSpeed);
        m_ConveyorControllerBottom.set(ControlMode.PercentOutput, -loadSpeed);
    }

    public void conveyorKill() {
        m_ConveyorControllerTop.set(ControlMode.PercentOutput, 0);
        m_ConveyorControllerBottom.set(ControlMode.PercentOutput, 0);
    }

}
