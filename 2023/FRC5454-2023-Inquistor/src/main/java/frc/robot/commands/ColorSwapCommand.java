package frc.robot.commands;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ChargedUp;
import frc.robot.Constants.LEDS;
import frc.robot.classes.LEDSChargedup;
import frc.robot.classes.LEDSChargedup.LEDMode;
import frc.robot.Constants;

public class ColorSwapCommand extends CommandBase{
    private LEDSChargedup m_LED;
    private LEDMode m_ledMode;
    public ColorSwapCommand(LEDSChargedup LED, LEDMode ledMode){
        m_LED=LED;
        m_ledMode=ledMode;  
    }

    @Override
    public void initialize(){
        m_LED.setRobotMode(m_ledMode);
        m_LED.updateLED();
    }

    @Override 
    public boolean isFinished(){
        m_LED.setPipelineLED();
        if(m_ledMode==LEDMode.AUTOSCORING){
            if(m_LED.m_canSeeTarget==true){
                m_LED.setRobotMode(LEDMode.CANSEETARGET);
            }else{
                m_LED.setRobotMode(LEDMode.CANTSEETARGET);
            }
        }
        return true;
    }

}
