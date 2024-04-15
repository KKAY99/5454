package frc.robot.subsystems;
import com.fasterxml.jackson.annotation.JsonTypeInfo.Id;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;

import org.littletonrobotics.junction.Logger;
import frc.robot.Constants;
import frc.robot.Constants.InputControllers;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.TurretConstants;
import frc.robot.utilities.AnalogAutoDirectFB6DN0E;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase{
    private TurretSubsystem m_turret;

    private CANSparkMax m_intakeOne;
    private CANSparkMax m_intakeTwo;
    private CANSparkMax m_intakeExtension;

    private double m_currentSpeed;

    private boolean m_intakeToggle=false;
    private AnalogAutoDirectFB6DN0E m_irReflector;
    private DigitalInput m_breakbeam;

    public IntakeSubsystem(TurretSubsystem turret,int motorOne,int motorTwo,int intakeExtension,int sensorPort){
        m_intakeOne= new CANSparkMax(motorOne,MotorType.kBrushless);
        m_intakeOne.setSmartCurrentLimit(Constants.k30Amp);
        m_intakeOne.setIdleMode(IdleMode.kBrake);
        m_intakeTwo= new CANSparkMax(motorTwo,MotorType.kBrushless);
        m_intakeTwo.setIdleMode(IdleMode.kBrake);
        m_intakeTwo.setSmartCurrentLimit(Constants.k30Amp);
        m_intakeOne.burnFlash(); //ensure all settings are saved if a a browout happens
        m_intakeTwo.burnFlash(); //ensure all settings are saved if a a browout happens
        m_breakbeam=new DigitalInput(sensorPort);
        m_intakeExtension=new CANSparkMax(intakeExtension,MotorType.kBrushless);
        m_intakeExtension.setSmartCurrentLimit(Constants.k30Amp);
        m_intakeExtension.setOpenLoopRampRate(0);
        m_intakeExtension.setClosedLoopRampRate(0);

        m_turret=turret;

        //TODO: REMOVE CONSTANT - EVIL CONSTANT
        //m_irReflector=new AnalogAutoDirectFB6DN0E(analogPort);
      //  m_TOFlow = new TimeOfFlight(55);
       // m_TOFlow.setRangingMode(RangingMode.Short, 24);
        m_intakeOne.setPeriodicFramePeriod(PeriodicFrame.kStatus3,1000);
        m_intakeOne.setPeriodicFramePeriod(PeriodicFrame.kStatus4,1000);
        m_intakeOne.setPeriodicFramePeriod(PeriodicFrame.kStatus5,1000);
        m_intakeOne.setPeriodicFramePeriod(PeriodicFrame.kStatus6,1000);
        m_intakeTwo.setPeriodicFramePeriod(PeriodicFrame.kStatus3,1000);
        m_intakeTwo.setPeriodicFramePeriod(PeriodicFrame.kStatus4,1000);
        m_intakeTwo.setPeriodicFramePeriod(PeriodicFrame.kStatus5,1000);
        m_intakeTwo.setPeriodicFramePeriod(PeriodicFrame.kStatus6,1000);
        
    }

    public void runIntakeExtension(double speed){
        double turretEncoderValue=m_turret.GetEncoderValue();

        if(turretEncoderValue<turretEncoderValue+TurretConstants.turretDeadBand&&
            turretEncoderValue>turretEncoderValue-TurretConstants.turretDeadBand){
            m_intakeExtension.set(speed);
        }else{
            if(speed<0){
                m_intakeExtension.set(speed);
            }else{
                m_intakeExtension.set(-speed);
            }
        }
    }

    public void stopIntakeExtension(){
        m_intakeExtension.set(0);
    }

    public void runIntake(double speed){
        m_currentSpeed=speed;
        m_intakeOne.set(speed);
        m_intakeTwo.set(speed);

        //If Outtaking outake the intake extension
        if(speed<0){
            runIntakeExtension(-IntakeConstants.intakeExtensionSpeed);
        }else{
            runIntakeExtension(IntakeConstants.intakeExtensionSpeed);
        }
    }

    public void stopIntake(){
        m_currentSpeed=0;
        m_intakeOne.set(0);
        m_intakeTwo.set(0);

        stopIntakeExtension();
    }

    public boolean isRunning(){
        return (m_currentSpeed>0.1);
    }
            
    public void setBrakeOn(){
        m_intakeOne.setIdleMode(IdleMode.kBrake);
        m_intakeTwo.setIdleMode(IdleMode.kBrake);
    
    } 

    public void setCoastOn(){
        m_intakeOne.setIdleMode(IdleMode.kCoast);
        m_intakeTwo.setIdleMode(IdleMode.kCoast);
    }

    public void ToggleIntake(double speed,XboxController xboxController){
        if(m_intakeToggle){
            m_intakeToggle=false;
            if(xboxController!=null){
                xboxController.setRumble(RumbleType.kBothRumble,InputControllers.kRumbleoff);
            }
            stopIntake();
        }else{
            m_intakeToggle=true;
            if(xboxController!=null){
                xboxController.setRumble(RumbleType.kBothRumble,Constants.InputControllers.kRumbleFull);
            }
            runIntake(speed);
        }
    }

    public void ResetToggleBoolean(){
        m_intakeToggle=false;
    }

    public boolean isBeamBroken(){
        boolean returnValue=false;
        if(m_breakbeam.get()){
            returnValue=false;
        }else{
            returnValue=true;
        }

     return returnValue; 
    }

    @Override
    public void periodic(){
        Logger.recordOutput("Intake/BeamBroken",isBeamBroken());
        //System.out.println(isBeamBroken());
        Logger.recordOutput("Intake/IntakeSpeed",m_currentSpeed);
    }
}
