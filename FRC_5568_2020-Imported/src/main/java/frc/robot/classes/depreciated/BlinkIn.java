/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.classes.depreciated;

import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

/**
 * This class controls the Blinkin Lights
 */
public class BlinkIn {

    // Create BlinkIn Spark
    Spark BlinkIn;

    // Create Configurable Values
    public NetworkTableEntry m_PwmOutput;

    /**
     * Initializes the BlinkIn controller and base settings.
     * 
     * @param blinkIn          The Spark that the BlinkIn is on.
     * @param defaultPwmOutput The default lighting code for the BlinkIn
     */
    public BlinkIn(Spark blinkIn, double defaultPwmOutput) {
        BlinkIn = blinkIn;
        m_PwmOutput = Shuffleboard.getTab("SubSystems").add("BlinkIn PWM Output", defaultPwmOutput)
                .withWidget("Number Slider").withPosition(2, 1).withSize(2, 1).getEntry();
    }

    /**
     * Updates the BlinkIn each loop.
     */
    public void update() {
        BlinkIn.set(m_PwmOutput.getDouble(0));
    }
}
