package frc.robot.common.drivers;

import edu.wpi.first.math.controller.PIDController;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.net.WPINetJNI;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import java.io.Console;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMax.IdleMode;
import frc.robot.subsystems.WPIDriveTrainSubsystem;
import frc.robot.common.drivers.MA3SwerveEncoder;

public class WPISwerveModule {
    private double m_wheelRadius=2.0;
    private double m_maxAngularSpeed=Math.PI;
    private double m_maxAngularAcceleration=2 * Math.PI;
    private int m_encoderResolution=4096;
    
    private CANSparkMax m_driveMotor;
    private CANSparkMax m_turningMotor; 

    private RelativeEncoder m_driveEncoder;
    private RelativeEncoder m_turningEncoder;

    private MA3SwerveEncoder m_absEncoder;

    private PIDController m_turningPIDController;

    private WPIDriveTrainSubsystem m_WPIDrive;

    public WPISwerveModule(WPIDriveTrainSubsystem WPIDrive,int driveMotorPort,int turningMotorPort, int absoluteEncoderPort){
        m_driveMotor=new CANSparkMax(driveMotorPort,MotorType.kBrushless);
        m_turningMotor=new CANSparkMax(turningMotorPort,MotorType.kBrushless);

        m_driveMotor.setIdleMode(IdleMode.kBrake);
        m_turningMotor.setIdleMode(IdleMode.kBrake);

        m_driveMotor.setSmartCurrentLimit(40);
        m_turningMotor.setSmartCurrentLimit(20);

        m_driveEncoder=m_driveMotor.getEncoder();
        m_turningEncoder=m_turningMotor.getEncoder();
        m_absEncoder=new MA3SwerveEncoder(absoluteEncoderPort,0);

        m_driveEncoder.getPositionConversionFactor();
        m_driveEncoder.getVelocityConversionFactor();

        m_turningEncoder.getPositionConversionFactor();
        m_turningEncoder.getVelocityConversionFactor();

        m_turningPIDController=new PIDController(0.5,0,0);
        m_turningPIDController.enableContinuousInput(-Math.PI,Math.PI);

        m_turningMotor.burnFlash();

        //KK Is Static Value
        //m_absEncoder.setDistancePerRotation(m_absEncoder. .getDistancePerRotation());
   
        m_driveMotor.setSmartCurrentLimit(40);
        m_turningMotor.setSmartCurrentLimit(20);

        m_WPIDrive=WPIDrive;

        resetDriveEncoder();
    }

    public double getDrivePosition(){
        return m_driveEncoder.getPosition();
    }

    public double getTurnPosition(){
      //  return m_turningEncoder.getPosition();
      //System.out.println("Wheel Angle "+ m_absEncoder.GetActualAngle());
      return m_absEncoder.GetActualAngle();
    }

    public double getDriveVelocity(){
        return m_driveEncoder.getVelocity();
    }

    public double getTurnVelocity(){
        return m_turningEncoder.getVelocity();
    }

    public SwerveModulePosition getPosition(){
        return(new SwerveModulePosition(getDrivePosition(),new Rotation2d(getTurnPosition())));
    }

    public double getABSEncoderRad(){
        double angle;

        //angle=1-m_absEncoder.getAbsolutePosition();
        //KK Updates 8/26/23 
        angle=1-m_absEncoder.GetActualPosition();
        
        angle*=2.0*Math.PI;

        angle-=Constants.WPISwerve.absoluteEncoderOffsetRad;

        return angle;
    }

    public void resetDriveEncoder(){
        m_driveEncoder.setPosition(0);
    }

    public void stop(){
        m_driveMotor.set(0);
        m_turningMotor.set(0);
    }

    public SwerveModuleState getState(){
        return new SwerveModuleState(getDriveVelocity(),new Rotation2d(getTurnPosition()));
    }

    public void setState(SwerveModuleState state){ 

        if(Math.abs(state.speedMetersPerSecond)<0.001){
            stop();
            return;
        }

        state=SwerveModuleState.optimize(state, getState().angle);

        m_driveMotor.set(state.speedMetersPerSecond/Constants.WPISwerve.physicalMaxSpeedMetersPerSecond*m_WPIDrive.speedMultiplier);

        //m_turningMotor.set(m_turningPIDController.calculate(getTurnPosition(),state.angle.getRadians()));
    
        double diff = getTurnPosition()-state.angle.getRadians();
        //System.out.println("Diff is "+ diff);
        if (Math.abs(diff)>5){
            if(diff<0){
                diff=Math.abs(diff);
                m_turningMotor.set(diff/360);
            } else {
                m_turningMotor.set(-diff/360);
            }
        } else{
            m_turningMotor.set(0);
        }
        //System.out.println(getTurnPosition() + " ** " + state.angle.getRadians());
        //System.out.println("Drive Value"+state.speedMetersPerSecond/Constants.WPISwerve.physicalMaxSpeedMetersPerSecond*m_WPIDrive.speedMultiplier);
        //System.out.println("Turning Value"+m_turningPIDController.calculate(getTurnPosition(),state.angle.getRadians()));
    }
}