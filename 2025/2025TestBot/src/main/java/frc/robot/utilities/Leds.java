// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utilities;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdleConfiguration;

import frc.robot.Constants.LedConstants.ColorState;
import frc.robot.Constants.LedConstants.LEDStates;


/** Add your docs here. */
public class Leds {
    private ColorState m_currentColorstate;
    private LEDStates m_currentState;
    private CANdle m_CANdle;
    public Leds(int CanID){
       m_CANdle = new CANdle(CanID);
       CANdleConfiguration CANdleConfig = new CANdleConfiguration();
       m_CANdle.getAllConfigs(CANdleConfig);
    }
    public void setLedState(LEDStates state){
        m_currentState = state;
        switch(m_currentState){
            case TELEOP: 
                setColorState(ColorState.GREEN);
                break;
            
            case DISABLED:
                setColorState(ColorState.RED);
                break;
            
        }


    }
       private void setColorState(ColorState color){
        m_currentColorstate = color;
        switch (m_currentColorstate) {
            case GREEN:
                m_CANdle.setLEDs(0,225,0);
                break;
            case PURPLE:
                m_CANdle.setLEDs(128,0,128);
                break;
            case RED:
                m_CANdle.setLEDs(225,0,0);
                break;
            case BLUE:
                m_CANdle.setLEDs(0,0,225);
                break;
            
        
        }
    } 
}
