// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utilities;
import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.FireAnimation;
import com.ctre.phoenix.led.LarsonAnimation;
import com.ctre.phoenix.led.LarsonAnimation.BounceMode;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.CANdle.LEDStripType;

import frc.robot.Constants.AnimationStates;
import frc.robot.Constants.ColorStates;
import frc.robot.Constants.LEDStates;


/** Add your docs here. */
public class Leds {
    private AnimationStates m_currentAnimationState;
    private ColorStates m_currentColorstate;
    private LEDStates m_currentState;
    private CANdle m_CANdle;
    
    private int m_ledCount;
    private int m_startIndex = 0;

    private Animation m_toAnimate = null;


    public Leds(int CanID, int ledCount){
       m_CANdle = new CANdle(CanID);
       m_ledCount = ledCount;
       m_CANdle.configLEDType(LEDStripType.GRB);
       m_CANdle.configBrightnessScalar(0.25);

    }

    public void setLedState(LEDStates state){
        setAnimationState(AnimationStates.NULL);
        m_currentState = state;
        switch(m_currentState){
            case TELEOP: 
                setColorState(ColorStates.WHITE);
                break;
            
            case DISABLED:
                setColorState(ColorStates.RED);
                break;

            case HASCORAL:
                setColorState(ColorStates.WHITE);
                break;

            case HASALGEA:
                setColorState(ColorStates.PURPLE);
                break;

            case LINEDUP:
                break;
            
            case SCORED:
                break;
            
        }


    }
    private void setColorState(ColorStates color){
        m_currentColorstate = color;
        switch (m_currentColorstate) {
            case GREEN:
                m_CANdle.setLEDs(0,225,0,0,m_startIndex, m_ledCount);
                break;
            case PURPLE:
                m_CANdle.setLEDs(128,0,128,0,m_startIndex, m_ledCount);
                break;
            case RED:
                m_CANdle.setLEDs(225,0,0,0,m_startIndex, m_ledCount);
                break;
            case BLUE:
                m_CANdle.setLEDs(0,0,225,0,m_startIndex, m_ledCount);
                break;
            case WHITE:
                m_CANdle.setLEDs(128,128,128,128, m_startIndex, m_ledCount);
                break;
        }
    }

    private void setAnimationState(AnimationStates animation){
        m_currentAnimationState = animation;
        switch (m_currentAnimationState) {
            case FIRE:
                m_toAnimate = new FireAnimation(0.25, 0.1, m_ledCount, 0.5, 0.9);
                break;
        
            case RAINBOW:
                m_toAnimate = new RainbowAnimation(0.25, 0.25, m_ledCount);
                break;

            case LARSON:
                m_toAnimate = new LarsonAnimation(0, 255, 46, 0, 1, m_ledCount, BounceMode.Front, 3);
                break;

            case NULL:
                m_toAnimate = null;
                break;

        
            
        }if (m_toAnimate != null){
            m_CANdle.animate(m_toAnimate);
            }else{
                m_CANdle.animate(m_toAnimate);
            }
    }


    
}
