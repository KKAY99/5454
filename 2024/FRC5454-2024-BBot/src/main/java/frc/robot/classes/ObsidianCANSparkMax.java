package frc.robot.classes;
import javax.lang.model.util.ElementScanner14;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.Timer;
public class ObsidianCANSparkMax extends CANSparkMax{

    /**
     * <p> 5454 CANSparkMaxWrapper
     * <p> Will set SmartCurrentLimit to 30
     * <p> Will set all Periodic Status 3-6 to 1000 ms
     * @param canID Motor ID
     * @param MotorType MotorType: Brushed or Brushless
    */
    public ObsidianCANSparkMax(int canID,MotorType motorType,boolean breakMode){
        super(canID,motorType);
        setSmartCurrentLimit(Constants.k30Amp); // default motors to 30 amp
        setPeriodicFramePeriod(PeriodicFrame.kStatus3,1000);
        setPeriodicFramePeriod(PeriodicFrame.kStatus4,1000);
        setPeriodicFramePeriod(PeriodicFrame.kStatus5,1000);
        setPeriodicFramePeriod(PeriodicFrame.kStatus6,1000); 
        setClosedLoopRampRate(0);
        setOpenLoopRampRate(0);
            if(breakMode){
            setIdleMode(IdleMode.kBrake);
        } else{
            setIdleMode(IdleMode.kCoast);
        }
        Timer.delay(0.5);   // delay due to rev bug on CAN bus when burning Flash 
        burnFlash();
        Timer.delay(0.5);   // delay due to rev bug on CAN bus when burning Flash 
    }

    /**
     * <p> 5454 CANSparkMaxWrapper
     * <p> Will set all Periodic Status 3-6 to 1000 ms
     * @param canID Motor ID
     * @param curentLimit SmartCurrentLimit for the Motor
     * @param MotorType MotorType: Brushed or Brushless
    */
    public ObsidianCANSparkMax(int canID,MotorType motorType,boolean breakMode,int currentLimit){  
        super(canID,motorType);
        setSmartCurrentLimit(currentLimit);
        setPeriodicFramePeriod(PeriodicFrame.kStatus3,1000);
        setPeriodicFramePeriod(PeriodicFrame.kStatus4,1000);
        setPeriodicFramePeriod(PeriodicFrame.kStatus5,1000);
        setPeriodicFramePeriod(PeriodicFrame.kStatus6,1000); 
        setClosedLoopRampRate(0);
        setOpenLoopRampRate(0);
        if(breakMode){
            setIdleMode(IdleMode.kBrake);
        } else{
            setIdleMode(IdleMode.kCoast);
        }
        Timer.delay(0.5);   // delay due to rev bug on CAN bus when burning Flash 
        burnFlash();
        Timer.delay(0.5);   // delay due to rev bug on CAN bus when burning Flash 
    }

    /**
     * <p> 5454 CANSparkMaxWrapper
     * @param canID Motor ID
     * @param MotorType MotorType: Brushed or Brushless
     * @param currentLimit SmartCurrentLimit for the Motor
     * @param periodicStatus3 Sets the Status Frame 3 rate in ms
     * @param periodicStatus4 Sets the Status Frame 4 rate in ms
     * @param periodicStatus5 Sets the Status Frame 3 rate in ms
     * @param periodicStatus6 Sets the Status Frame 3 rate in ms
    */
    public ObsidianCANSparkMax(int canID,MotorType motorType,boolean breakMode,int currentLimit,int periodicStatus3,
                            int periodicStatus4,int periodicStatus5,int periodicStatus6){
        super(canID,motorType);
        setSmartCurrentLimit(currentLimit);
        setPeriodicFramePeriod(PeriodicFrame.kStatus3,periodicStatus3);
        setPeriodicFramePeriod(PeriodicFrame.kStatus4,periodicStatus4);
        setPeriodicFramePeriod(PeriodicFrame.kStatus5,periodicStatus5);
        setPeriodicFramePeriod(PeriodicFrame.kStatus6,periodicStatus6); 
        setClosedLoopRampRate(0);
        setOpenLoopRampRate(0); 
        if(breakMode){
            setIdleMode(IdleMode.kBrake);
        } else{
            setIdleMode(IdleMode.kCoast);
        }
        Timer.delay(0.5);   // delay due to rev bug on CAN bus when burning Flash 
        burnFlash();
        Timer.delay(0.5);   // delay due to rev bug on CAN bus when burning Flash 
   
    }
}
