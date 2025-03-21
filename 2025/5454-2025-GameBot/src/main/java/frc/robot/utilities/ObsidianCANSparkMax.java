package frc.robot.utilities;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import frc.robot.Constants;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.config.SparkBaseConfig;
import edu.wpi.first.wpilibj.Timer;

public class ObsidianCANSparkMax extends SparkBase{

    /**
     * <p> 5454 CANSparkMaxWrapper
     * <p> Will set SmartCurrentLimit to 30
     * <p> Will set all Periodic Status 3-6 to 1000 ms
     * @param canID Motor ID
     * @param MotorType MotorType: Brushed or Brushless
    */
    public ObsidianCANSparkMax(int canID,MotorType motorType,boolean breakMode){
        super(canID,motorType,SparkModel.SparkMax);
        try{
            SparkBaseConfig newConfig =new SparkMaxConfig();
            newConfig.inverted(false);
            newConfig.smartCurrentLimit(Constants.k30Amp);
            newConfig.signals.motorTemperaturePeriodMs(1000);
            newConfig.signals.busVoltagePeriodMs(1000);
            newConfig.signals.analogPositionPeriodMs(1000);
            newConfig.signals.analogVelocityPeriodMs(1000);
            newConfig.signals.analogVoltagePeriodMs(1000);
            newConfig.signals.outputCurrentPeriodMs(1000);
            newConfig.signals.externalOrAltEncoderPosition(1000);
            newConfig.signals.externalOrAltEncoderVelocity(1000);

            if(breakMode){
                newConfig.idleMode(IdleMode.kBrake);
            } else{
                newConfig.idleMode(IdleMode.kBrake);
            }
            Timer.delay(0.5);   // delay due to rev bug on CAN bus when burning Flash 
            configure(newConfig,SparkBase.ResetMode.kResetSafeParameters,SparkBase.PersistMode.kNoPersistParameters);
            Timer.delay(0.5);   // delay due to rev bug on CAN bus when burning Flash 
        }
    catch (Exception e){
        System.out.println("Exception in Creating CAN ID :" +  canID);
        System.out.println(e.toString());
        }
    }

    /**
     * <p> 5454 CANSparkMaxWrapper
     * <p> Will set all Periodic Status 3-6 to 1000 ms
     * @param canID Motor ID
     * @param curentLimit SmartCurrentLimit for the Motor
     * @param MotorType MotorType: Brushed or Brushless
    */
    public ObsidianCANSparkMax(int canID,MotorType motorType,boolean breakMode,int currentLimit){  
        super(canID,motorType,SparkModel.SparkMax);
        SparkBaseConfig newConfig =new SparkMaxConfig();
        newConfig.inverted(false);
        newConfig.smartCurrentLimit(currentLimit);
        newConfig.signals.motorTemperaturePeriodMs(1000);
        newConfig.signals.busVoltagePeriodMs(1000);
        newConfig.signals.analogPositionPeriodMs(1000);
        newConfig.signals.analogVelocityPeriodMs(1000);
        newConfig.signals.analogVoltagePeriodMs(1000);
        newConfig.signals.outputCurrentPeriodMs(1000);
        newConfig.signals.externalOrAltEncoderPosition(1000);
        newConfig.signals.externalOrAltEncoderVelocity(1000);

        if(breakMode){
            newConfig.idleMode(IdleMode.kBrake);
        } else{
            newConfig.idleMode(IdleMode.kBrake);
        }
        Timer.delay(0.5);   // delay due to rev bug on CAN bus when burning Flash 
        configure(newConfig,SparkBase.ResetMode.kResetSafeParameters,SparkBase.PersistMode.kNoPersistParameters);
        Timer.delay(0.5);   // delay due to rev bug on CAN bus when burning Flash 
    }

    /**
     * <p> 5454 CANSparkMaxWrapper
     * <p> Will set all Periodic Status 3-6 to 1000 ms
     * @param canID Motor ID
     * @param curentLimit SmartCurrentLimit for the Motor
     * @param MotorType MotorType: Brushed or Brushless
     * @param periodicStatus1Ms
     * @param periodicStatus2Ms
    */
    public ObsidianCANSparkMax(int canID,MotorType motorType,boolean breakMode,int currentLimit,int periodicStatus1Ms,
                                int periodicStatus2Ms){  
        super(canID,motorType,SparkModel.SparkMax);
        SparkBaseConfig newConfig =new SparkMaxConfig();
        newConfig.inverted(false);
        newConfig.smartCurrentLimit(currentLimit);
        newConfig.signals.motorTemperaturePeriodMs(1000);
        newConfig.signals.busVoltagePeriodMs(1000);
        newConfig.signals.analogPositionPeriodMs(1000);
        newConfig.signals.analogVelocityPeriodMs(1000);
        newConfig.signals.analogVoltagePeriodMs(1000);
        newConfig.signals.outputCurrentPeriodMs(1000);
        newConfig.signals.externalOrAltEncoderPosition(1000);
        newConfig.signals.externalOrAltEncoderVelocity(1000);

        if(breakMode){
            newConfig.idleMode(IdleMode.kBrake);
        } else{
            newConfig.idleMode(IdleMode.kBrake);
        }
        Timer.delay(0.5);   // delay due to rev bug on CAN bus when burning Flash 
        configure(newConfig,SparkBase.ResetMode.kResetSafeParameters,SparkBase.PersistMode.kNoPersistParameters);
        Timer.delay(0.5);   // delay due to rev bug on CAN bus when burning Flash 
    }

    /**
     * <p> 5454 CANSparkMaxWrapper
     * @param canID Motor ID
     * @param MotorType MotorType: Brushed or Brushless
     * @param currentLimit SmartCurrentLimit for the Motor
     * @param p Sets Closed Loop P
     * @param i Sets Closed Loop I
     * @param d Sets Closed Loop D
    */
    public ObsidianCANSparkMax(int canID,MotorType motorType,boolean breakMode,int currentLimit,double p,double i,double d){
    super(canID,motorType,SparkModel.SparkMax);
        SparkBaseConfig newConfig =new SparkMaxConfig();
        newConfig.inverted(false);
        newConfig.smartCurrentLimit(Constants.k30Amp);
        newConfig.closedLoop.p(p);
        newConfig.closedLoop.i(i);
        newConfig.closedLoop.d(d);
        newConfig.signals.motorTemperaturePeriodMs(1000);
        newConfig.signals.busVoltagePeriodMs(1000);
        newConfig.signals.analogPositionPeriodMs(1000);
        newConfig.signals.analogVelocityPeriodMs(1000);
        newConfig.signals.analogVoltagePeriodMs(1000);
        newConfig.signals.outputCurrentPeriodMs(1000);
        newConfig.signals.externalOrAltEncoderPosition(1000);
        newConfig.signals.externalOrAltEncoderVelocity(1000);
        
        if(breakMode){
            newConfig.idleMode(IdleMode.kBrake);
        } else{
            newConfig.idleMode(IdleMode.kBrake);
        }
        Timer.delay(0.5);   // delay due to rev bug on CAN bus when burning Flash 
        configure(newConfig,SparkBase.ResetMode.kResetSafeParameters,SparkBase.PersistMode.kNoPersistParameters);
        Timer.delay(0.5);   // delay due to rev bug on CAN bus when burning Flash 
   
    }

    /**
     * <p> 5454 CANSparkMaxWrapper
     * @param canID Motor ID
     * @param MotorType MotorType: Brushed or Brushless
     * @param currentLimit SmartCurrentLimit for the Motor
     * @param p Sets Closed Loop P
     * @param i Sets Closed Loop I
     * @param d Sets Closed Loop D
     * @param maxAndMin Sets Max and Min Percent Output
    */
    public ObsidianCANSparkMax(int canID,MotorType motorType,boolean breakMode,int currentLimit,double p,double i,double d,
                                double maxAndMin){
        super(canID,motorType,SparkModel.SparkMax);
        try{
            SparkBaseConfig newConfig =new SparkMaxConfig();
            newConfig.inverted(false);
            newConfig.smartCurrentLimit(currentLimit);
            newConfig.closedLoop.p(p);
            newConfig.closedLoop.i(i);
            newConfig.closedLoop.d(d);
            newConfig.closedLoop.maxOutput(maxAndMin);
            newConfig.closedLoop.minOutput(-maxAndMin);
            newConfig.signals.motorTemperaturePeriodMs(1000);
            newConfig.signals.busVoltagePeriodMs(1000);
            newConfig.signals.analogPositionPeriodMs(1000);
            newConfig.signals.analogVelocityPeriodMs(1000);
            newConfig.signals.analogVoltagePeriodMs(1000);
            newConfig.signals.outputCurrentPeriodMs(1000);
            newConfig.signals.externalOrAltEncoderPosition(1000);
            newConfig.signals.externalOrAltEncoderVelocity(1000);
                
            if(breakMode){
                newConfig.idleMode(IdleMode.kBrake);
            } else{
                newConfig.idleMode(IdleMode.kCoast);
            }
            Timer.delay(0.5);   // delay due to rev bug on CAN bus when burning Flash 
            configure(newConfig,SparkBase.ResetMode.kResetSafeParameters,SparkBase.PersistMode.kNoPersistParameters);
            Timer.delay(0.5);   // delay due to rev bug on CAN bus when burning Flash 
       
        }
        catch (Exception e){
            System.out.println("Exception in Creating CAN ID :" +  canID);
            System.out.println(e.toString());
        }
        }
    /**
     * <p> 5454 CANSparkMaxWrapper
     * @param canID Motor ID
     * @param MotorType MotorType: Brushed or Brushless
     * @param currentLimit SmartCurrentLimit for the Motor
     * @param pK1 Sets Closed Loop P Slot 1
     * @param iK1 Sets Closed Loop I Slot 1
     * @param dK1 Sets Closed Loop D Slot 1
     * @param maxAndMin Sets Max and Min Percent Output Slot 1
     * @param pK2 Sets Closed Loop P Slot 2
     * @param iK2 Sets Closed Loop I Slot 2
     * @param dK2 Sets Closed Loop D Slot 2
     * @param maxAndMinK2 Sets Max and Min Percent Output Slot 2
    */
    public ObsidianCANSparkMax(int canID,MotorType motorType,boolean breakMode,int currentLimit,double pK1,double iK1,double dK1,
                                double maxAndMinK1,double pK2,double iK2,double dK2,double maxAndMinK2){
        super(canID,motorType,SparkModel.SparkMax);
        SparkBaseConfig newConfig =new SparkMaxConfig();
        newConfig.inverted(false);
        newConfig.smartCurrentLimit(currentLimit);
        newConfig.closedLoop.p(pK1,ClosedLoopSlot.kSlot0);
        newConfig.closedLoop.i(iK1,ClosedLoopSlot.kSlot0);
        newConfig.closedLoop.d(dK1,ClosedLoopSlot.kSlot0);
        newConfig.closedLoop.maxOutput(maxAndMinK1,ClosedLoopSlot.kSlot0);
        newConfig.closedLoop.minOutput(-maxAndMinK1,ClosedLoopSlot.kSlot0);
        newConfig.closedLoop.p(pK2,ClosedLoopSlot.kSlot1);
        newConfig.closedLoop.i(iK2,ClosedLoopSlot.kSlot1);
        newConfig.closedLoop.d(dK2,ClosedLoopSlot.kSlot1);
        newConfig.closedLoop.maxOutput(maxAndMinK2,ClosedLoopSlot.kSlot1);
        newConfig.closedLoop.minOutput(-maxAndMinK2,ClosedLoopSlot.kSlot1);
        newConfig.signals.motorTemperaturePeriodMs(1000);
        newConfig.signals.busVoltagePeriodMs(1000);
        newConfig.signals.analogPositionPeriodMs(1000);
        newConfig.signals.analogVelocityPeriodMs(1000);
        newConfig.signals.analogVoltagePeriodMs(1000);
        newConfig.signals.outputCurrentPeriodMs(1000);
        newConfig.signals.externalOrAltEncoderPosition(1000);
        newConfig.signals.externalOrAltEncoderVelocity(1000);

        if(breakMode){
        newConfig.idleMode(IdleMode.kBrake);
        } else{
        newConfig.idleMode(IdleMode.kCoast);
        }
        Timer.delay(0.5);   // delay due to rev bug on CAN bus when burning Flash 
        configure(newConfig,SparkBase.ResetMode.kResetSafeParameters,SparkBase.PersistMode.kNoPersistParameters);
        Timer.delay(0.5);   // delay due to rev bug on CAN bus when burning Flash 

    }
}
