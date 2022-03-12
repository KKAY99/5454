/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.classes;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Solenoid;

/**
 * This class runs the subsystems for the 2019 game robot.
 */
public class SubSystems {

    // Create Solenoid
    private Solenoid m_hatcherExtend;
    private Solenoid m_hatcherGrab;

    public Boolean g_hatcherExtended = false;
    public Boolean g_hatcherGrabbing = false;

    /**
     * This initializes all of the motors and base settings for the robot subsystems
     * 
     * @param ClimbFront The motor for the Talon front climber
     * @param ClimbBack  The motor for the Talon back climber
     * @param ClimbDrive The Talon drive motor on the climber
     * @param Lift       The CANSparkMax Lift Motor
     * @param Intake     The intake motor
     * @param Hatcher    The solenoid for the Hatcher system
     */
    public SubSystems(Solenoid HatcherGrab, Solenoid HatcherExtend) {
        m_hatcherGrab = HatcherGrab;
        m_hatcherExtend = HatcherExtend;
    }

    // #region State Managers
    public void subsystemsStateManager(String systemInput, String stateInput) {
        subsystemsStateManager(systemInput, stateInput, null);
    }

    public void subsystemsStateManager(String systemInput, String stateInput, Double speed) {
        switch (systemInput) {
        case "hatchIntake":
            switch (stateInput) {
            case "Extend":
                hatchMechanismStateManager("Extend");
                break;

            case "Retract":
                hatchMechanismStateManager("Retract");

            case "Grab":
                hatchMechanismStateManager("Grab");

            case "Release":
                hatchMechanismStateManager("Release");

            default:
                printError("No state found - " + stateInput + " - for Hatch intake in subsystems manager.");
                break;
            }
            break;
        default:
            printError("There is no SubSystem by that name.");
            break;
        }
    }

    public void hatchMechanismStateManager(String input) {
        switch (input) {
        case "Extend":
            hatcherExtend();
            break;

        case "Retract":
            hatcherRetract();
            break;

        case "Grab":
            hatcherGrab();
            break;

        case "Release":
            hatcherRelease();
            break;

        default:
            printError("Illegal value for hatchMechanismStateManager - " + input);
            hatcherRelease();
            hatcherRetract();
            break;
        }
    }

    // #region Hatch Panel Actions
    public void hatcherExtend() {
        g_hatcherExtended = true;
    }

    public void hatcherRetract() {
        g_hatcherExtended = false;
    }

    public void hatcherGrab() {
        g_hatcherGrabbing = true;
    }

    public void hatcherRelease() {
        g_hatcherGrabbing = false;
    }
    // #endregion

    public void updatePhysicalState() {
        if (g_hatcherExtended) {
            m_hatcherExtend.set(true);
        } else {
            m_hatcherExtend.set(false);
        }
        if (g_hatcherGrabbing) {
            m_hatcherGrab.set(true);
        } else {
            m_hatcherGrab.set(false);
        }
    }

    public void printError(String error) {
        DriverStation.reportError(error, false);
    }
}
