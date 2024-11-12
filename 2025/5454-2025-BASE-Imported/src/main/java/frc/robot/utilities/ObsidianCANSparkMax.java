package frc.robot.utilities;
import javax.lang.model.util.ElementScanner14;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import frc.robot.Constants;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.config.SparkBaseConfig;
import edu.wpi.first.wpilibj.Timer;
import com.revrobotics.spark.SparkLowLevel.PeriodicFrame;

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
        setInverted(true);
        newConfig.smartCurrentLimit(Constants.k30Amp);
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
        setInverted(true);
        newConfig.smartCurrentLimit(Constants.k30Amp);
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
     * @param periodicStatus3 Sets the Status Frame 3 rate in ms
     * @param periodicStatus4 Sets the Status Frame 4 rate in ms
     * @param periodicStatus5 Sets the Status Frame 3 rate in ms
     * @param periodicStatus6 Sets the Status Frame 3 rate in ms
    */
    public ObsidianCANSparkMax(int canID,MotorType motorType,boolean breakMode,int currentLimit,int periodicStatus3,
                            int periodicStatus4,int periodicStatus5,int periodicStatus6){
    super(canID,motorType,SparkModel.SparkMax);
        SparkBaseConfig newConfig =new SparkMaxConfig();
        setInverted(true);
        newConfig.smartCurrentLimit(Constants.k30Amp);
        if(breakMode){
            newConfig.idleMode(IdleMode.kBrake);
        } else{
            newConfig.idleMode(IdleMode.kBrake);
        }
        Timer.delay(0.5);   // delay due to rev bug on CAN bus when burning Flash 
        configure(newConfig,SparkBase.ResetMode.kResetSafeParameters,SparkBase.PersistMode.kNoPersistParameters);
        Timer.delay(0.5);   // delay due to rev bug on CAN bus when burning Flash 
   
    }
}
