/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.classes.mechanisms;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

/**
 * This class is used to run swerve drive.
 */
public class Climber {
    // Create TalonSRX Speed Controllers
    private TalonSRX m_ClimbPullMotor;
    private VictorSPX m_ClimbRaiseMotor;

    // Create Variables
    public static final double MaxPullSpeed = 1.0;
    public static final double MaxRaiseSpeed = 1.0;

    /**
     * Configures the Conveyor
     * 
     * @param ClimbPullMotor  The TalonSRX for the motor that pulls the robot up
     * @param ClimbRaiseMotor The VictorSPX for the motor that raises the hook up
     */

    public Climber(TalonSRX ClimbPullMotor) {

        m_ClimbPullMotor = ClimbPullMotor;
        ClimbPullMotor.configFactoryDefault();
        m_ClimbPullMotor.setInverted(false);
    }

    /**
     * Configures the Conveyor
     * 
     * @param ClimbPullMotor  The TalonSRX for the motor that pulls the robot up
     * @param ClimbRaiseMotor The VictorSPX for the motor that raises the hook up
     */

    public Climber(TalonSRX ClimbPullMotor, VictorSPX ClimbRaiseMotor) {

        m_ClimbPullMotor = ClimbPullMotor;
        m_ClimbRaiseMotor = ClimbRaiseMotor;
        ClimbPullMotor.configFactoryDefault();
        ClimbRaiseMotor.configFactoryDefault();
        m_ClimbPullMotor.setInverted(false);
        m_ClimbRaiseMotor.setInverted(false);
    }

    // CLIMBS CLIMBS CLIMBS
    public void CLIMBCLIMBCLIMB() {
        CLIMBCLIMBCLIMB(MaxPullSpeed);
    }

    public void CLIMBCLIMBCLIMB(double power) {
        m_ClimbPullMotor.set(ControlMode.PercentOutput, power);
    }

    public void climbRaise(double power) {
        m_ClimbRaiseMotor.set(ControlMode.PercentOutput, power);
    }

    public void climbRaiseKill() {
        m_ClimbRaiseMotor.set(ControlMode.PercentOutput, 0);
    }

    public void climbPullKill() {
        m_ClimbPullMotor.set(ControlMode.PercentOutput, 0);
    }

    public void climbKill() {
        climbPullKill();
        climbRaiseKill();
    }

}
