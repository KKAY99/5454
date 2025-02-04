package frc.robot.utilities;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import frc.robot.Constants;
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
        SparkBaseConfig newConfig =new SparkMaxConfig();
        newConfig.inverted(false);
        newConfig.smartCurrentLimit(Constants.k30Amp);
        newConfig.signals.motorTemperaturePeriodMs(1000);
        newConfig.signals.busVoltagePeriodMs(1000);
        newConfig.signals.analogPositionPeriodMs(1000);
        newConfig.signals.analogVelocityPeriodMs(1000);
        newConfig.signals.analogVoltagePeriodMs(1000);

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
