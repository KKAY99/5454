package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.EncoderType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.Constants;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

public class RotateArmSubsystem extends SubsystemBase{

    private CANSparkMax m_rotateMotor;
    private GenericEntry rotateABSPos = Shuffleboard.getTab("EncoderValues").add("Rotate_ABS", 0.0).getEntry();
    //SparkMaxPIDController m_pidController;
    private DutyCycleEncoder m_absEncoder;
    //private RelativeEncoder m_rotateEncoder;
    private double m_rotatePos;

    public RotateArmSubsystem(int rotateMotor,int absEncoderPort){
        m_rotateMotor=new CANSparkMax(rotateMotor, MotorType.kBrushed);
        m_rotateMotor.setSmartCurrentLimit(40);
        m_rotateMotor.setIdleMode(IdleMode.kBrake);
        m_absEncoder=new DutyCycleEncoder(absEncoderPort);
        //m_rotateEncoder=m_rotateMotor.getEncoder();

        /*m_pidController = m_rotateMotor.getPIDController();
        m_pidController.setFeedbackDevice(m_rotateEncoder);   
        m_pidController.setP(Constants.Rotate.KP);
        m_pidController.setI(Constants.Rotate.KI);
        m_pidController.setD(Constants.Rotate.KD);
        m_pidController.setIZone(Constants.Rotate.KIZ);
        m_pidController.setFF(Constants.Rotate.KFF);
        m_pidController.setOutputRange(Constants.Rotate.minOutPut, Constants.Rotate.maxOutput);
    */
    }

    public void runWithLimits(double speed){
        if(hasHitBWDSoftLimit()==false&&hasHitFWDSoftLimit()==false){
            m_rotateMotor.set(speed);
        }else{
            if(hasHitBWDSoftLimit()==true){
                if(speed>0){
                    m_rotateMotor.set(speed);
                }else{
                    m_rotateMotor.set(0);
                }
            }else{
                if(hasHitFWDSoftLimit()==true){
                    if(speed<0){
                        m_rotateMotor.set(speed);
                    }
                }else{
                    m_rotateMotor.set(0);
                }
           }
        }
    }

    public void runWithOutLimits(double speed){
        m_rotateMotor.set(speed);
    }

    public void stop(){
        m_rotateMotor.stopMotor();
    }

    public double getRotatePos(){
        //return m_rotateEncoder.getPosition();
        return 1;
    }

    public double getABSPos(){
        double currentPos=m_absEncoder.getAbsolutePosition();
        if(currentPos<0.20){
            //BAD BAD HACK FOR what apepars to be hitting 1.0 point
            currentPos=currentPos+1;
        }
        return currentPos;
    }

    public boolean hasHitFWDSoftLimit(){
        boolean hasHitLimit=false;

        if(getABSPos()>Constants.RotateArm.FWDSoftLimitABS&&getABSPos()<0.075){
            hasHitLimit=true;
        }

        return hasHitLimit;
    }

    public boolean hasHitBWDSoftLimit(){
        boolean hasHitLimit=false;

        if(getABSPos()<Constants.RotateArm.BWDSoftLimitABS&&getABSPos()>0.35){
            hasHitLimit=true;
        }

        return hasHitLimit;
    }

    public void SetPosAndMove(double rotatePos){
        //m_pidController.setReference(rotatePos, CANSparkMax.ControlType.kPosition);
        m_rotatePos = rotatePos;
    }

    @Override
    public void periodic(){
        rotateABSPos.setDouble(getABSPos());
    }
}
