package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.io.Console;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.WPI_CANCoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import frc.robot.Constants;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.utilities.Limelight;
import frc.robot.utilities.ShotTable;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.controls.VelocityVoltage;
public class ShooterSubsystem extends SubsystemBase{
    private Limelight m_limeLight;

    private TalonFX m_ShootingMotor1;
    private TalonFX m_ShootingMotor2; 
    private CANSparkMax m_feederMotor;
    private CANSparkMax m_angleMotor;
    
    private SparkMaxPIDController m_anglePID;

    private RelativeEncoder m_angleEncoder;
    private WPI_CANCoder m_canCoder;
     
    private double m_baseSpeed;
    private double m_distance;

    private double m_targetAngle=0;
    private double m_shotsTaken;
    private double m_desiredVeloc1;
    private double m_desiredVeloc2;

    public ShooterSubsystem(Limelight limeLight,int shootingMotor1,int shootingMotor2,int angleMotor,int feedMotor,int canCoderId,double baseSpeed,boolean shouldUseDashBoardVals){
        m_limeLight=limeLight;
        m_baseSpeed=baseSpeed;
        m_ShootingMotor1=new TalonFX(shootingMotor1);  
        m_ShootingMotor2=new TalonFX(shootingMotor2);
        configmotor(m_ShootingMotor1);
        configmotor(m_ShootingMotor2);
        m_feederMotor=new CANSparkMax(feedMotor,MotorType.kBrushless);
        m_feederMotor.setSmartCurrentLimit(Constants.k30Amp);
        m_angleMotor = new CANSparkMax(angleMotor,MotorType.kBrushless);
        m_angleMotor.setSmartCurrentLimit(Constants.k30Amp);
        m_anglePID = m_angleMotor.getPIDController();
        m_canCoder=new WPI_CANCoder(canCoderId);
        m_angleEncoder=m_angleMotor.getEncoder();
    
        double anglekP = 0.8;//0.1 
        double anglekI = 0.00;
        double anglekD = 0.00; 
        double anglekIz = 0; 
        double anglekFF = 0.000015; 
        double anglekMaxOutput = 1; 
        double anglekMinOutput = -1;
    
        m_anglePID.setP(anglekP);
        m_anglePID.setI(anglekI);
        m_anglePID.setD(anglekD);
        m_anglePID.setIZone(anglekIz);
        m_anglePID.setFF(anglekFF);
        m_anglePID.setOutputRange(anglekMinOutput, anglekMaxOutput);
        m_angleMotor.getEncoder();


    }

    public void configmotor(TalonFX motor){
        var fx_cfg = new TalonFXConfiguration();
        // fetch *all* configs currently applied to the device
        var slot0Configs = new Slot0Configs();
        slot0Configs.kV = 0.12;
        slot0Configs.kP=0.11;
        slot0Configs.kI=0.48;
        slot0Configs.kD=0.01; 
        // apply gains, 50 ms totl timeout
        motor.getConfigurator().apply(slot0Configs, 0.050); 
        /**
        * Configure the current limits that will be used
        * Stator Current is the current that passes through the motor stators.
        *  Use stator current limits to limit rotor acceleration/heat production
        * Supply Current is the current that passes into the controller from the supply
        *  Use supply current limits to prevent breakers from tripping
        *
        * 
        //will need to use                                                               enabled | Limit(amp) | Trigger Threshold(amp) | Trigger Threshold Time(s)  */
        //fx_cfg.CurrentLimits.
        //motor.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true,      20,                25,                1.0));
        //motor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true,      10,                15,                0.5));
            
    }
    
    public void RunShootingMotors(double veloc1,double veloc2){
        //values are for the indivudal shooter motors
        m_desiredVeloc1=veloc1;
        m_desiredVeloc2=veloc2;

        VelocityVoltage m_velocity = new VelocityVoltage(0);
        m_velocity.Slot = 0;
        
        m_ShootingMotor1.setControl(m_velocity.withVelocity(veloc1));
        m_ShootingMotor2.setControl(m_velocity.withVelocity(veloc2));

        if(veloc1>0){
            m_feederMotor.set(-ShooterConstants.feederSpeed);
        }else{
            m_feederMotor.set(ShooterConstants.feederSpeed);
       }
    } 

    public void RunFeedRollers(double speed){
        m_feederMotor.set(speed);
    }

    public void StopFeedRollers(){
        m_feederMotor.set(0);
    }

    public void SlowShootingMotors(){
        //keep motors running
         VelocityVoltage m_velocity = new VelocityVoltage(0);
        m_velocity.Slot = 0;
        m_ShootingMotor1.setControl(m_velocity.withVelocity(m_baseSpeed));
        m_ShootingMotor2.setControl(m_velocity.withVelocity(m_baseSpeed));
      
        m_feederMotor.set(0);
    }
    public void stopShooter(){
        m_ShootingMotor1.set(0);
        m_ShootingMotor2.set(0);
        m_feederMotor.set(0);
        
    }

    public boolean isMotorVelocitysAtDesiredSpeed(double veloc1,double veloc2){
        boolean returnValue=false;

        System.out.println("Veloc1 "+veloc1);
        System.out.println("CurrentVeloc "+ GetVelocityMotor1());

        if(Math.abs(GetVelocityMotor1())+Math.abs(ShooterConstants.baseSpeedDeadband)>=Math.abs(veloc1)&&Math.abs(GetVelocityMotor1())-Math.abs(ShooterConstants.baseSpeedDeadband)<=Math.abs(veloc1)
            &&Math.abs(GetVelocityMotor2())+Math.abs(ShooterConstants.baseSpeedDeadband)>=Math.abs(veloc2)&&Math.abs(GetVelocityMotor2())-Math.abs(ShooterConstants.baseSpeedDeadband)<=Math.abs(veloc2)){
            returnValue=true;
        }

        return returnValue;
    }

    public double GetVelocityMotor1(){
        System.out.println(m_ShootingMotor1.getVelocity() + " = " + m_ShootingMotor1.getRotorVelocity());
        return m_ShootingMotor1.getRotorVelocity().getValueAsDouble();
        
    }

    public double GetVelocityMotor2(){
        return m_ShootingMotor2.getRotorVelocity().getValueAsDouble();
    }

    public void OutPutDistance(){
        ShotTable shotTable = new ShotTable();
        double calculation=shotTable.getVelocity1(m_limeLight.getDistance());
        System.out.println("Distance Calucations: "+calculation);
    }

    public void RotateShooter(double speed){
        m_angleMotor.set(speed);
    }

    public void stopRotate(){
        m_anglePID.setReference(0,ControlType.kVelocity);
        m_angleMotor.set(0);
    }

    public void setAngle(double targetAngle){
        m_targetAngle=targetAngle;
        m_anglePID.setReference(targetAngle,ControlType.kPosition);

    }

    public double getCanCoderPosition(){
        return m_canCoder.getAbsolutePosition();
    }

    public void zeroRelativePosition(){
        m_angleEncoder.setPosition(0);
    }

    public void zeroCanCoderPosition(){
        m_canCoder.setPosition(0);
    }

    public double getRelativePosition(){
        return m_angleEncoder.getPosition();
    }

    public void ResetControlType(){
        m_anglePID.setReference(0,ControlType.kVelocity);
    }

    public void setBrakeOn(){
        m_ShootingMotor1.setNeutralMode(NeutralModeValue.Brake);
        m_ShootingMotor1.setNeutralMode(NeutralModeValue.Brake);
        //m_angleMotor.setIdleMode(IdleMode.kBrake);
    } 
  
    public void setCoastOn(){
        m_ShootingMotor1.setNeutralMode(NeutralModeValue.Coast);
        m_ShootingMotor1.setNeutralMode(NeutralModeValue.Coast);
        //m_angleMotor.setIdleMode(IdleMode.kCoast);
    }

    public void ResetShotsTaken(){
        m_shotsTaken=0;
    }

    public void ShotTaken(){
        m_shotsTaken+=1;
    }

    public void PutTableMultiplier(){
    }

    public boolean isAtPodiumShot(){
        boolean returnValue=false;

        if(getRelativePosition()>Math.abs(Constants.ShooterConstants.podiumShotEncoderVal)-Constants.ShooterConstants.shooterPosDeadband&&
            getRelativePosition()<Math.abs(Constants.ShooterConstants.podiumShotEncoderVal)+Constants.ShooterConstants.shooterPosDeadband){
                returnValue=true;
        }

        return returnValue;
    }

    public boolean isAtMidShot(){
        boolean returnValue=false;

        if(getRelativePosition()>Math.abs(Constants.ShooterConstants.midShotEncoderVal)-Constants.ShooterConstants.shooterPosDeadband&&
            getRelativePosition()<Math.abs(Constants.ShooterConstants.midShotEncoderVal)+Constants.ShooterConstants.shooterPosDeadband){
                returnValue=true;
        }

        return returnValue;
    }

    public boolean isAtShortShot(){
        boolean returnValue=false;

        if(getRelativePosition()>Math.abs(Constants.ShooterConstants.shortShotEncoderVal)-Constants.ShooterConstants.shooterPosDeadband&&
            getRelativePosition()<Math.abs(Constants.ShooterConstants.shortShotEncoderVal)+Constants.ShooterConstants.shooterPosDeadband){
                returnValue=true;
        }

        return returnValue;
    }

    public boolean hasHitRotateLimit(double speed){
        boolean returnValue=false;
        if(speed>0){
            //if going up in speed and hitting HighSoftLimit then return true
            returnValue=(getRelativePosition()>=Constants.ShooterConstants.rotateHighSoftLimit);
        }else{
            //negatives speed so check low soft limit 
            returnValue=(getRelativePosition()<=Constants.ShooterConstants.rotateLowSoftLimit);
        }
        return returnValue;
    }

    @Override
    public void periodic(){
        Logger.recordOutput("Shooter/Shooter1VelocitySet",m_desiredVeloc1);
        Logger.recordOutput("Shooter/Shooter2VelocitySet",m_desiredVeloc2);
        Logger.recordOutput("Shooter/ShooterSetAngle",m_targetAngle);
      //  Logger.recordOutput("Shooter/ShootMotor1Velocity",GetVelocityMotor1());
      //  Logger.recordOutput("Shooter/ShootMotor2Velocity",GetVelocityMotor2());
      //  Logger.recordOutput("Shooter/TalonMotor1Temp",m_ShootingMotor1.getDeviceTemp().getValueAsDouble());
      //  Logger.recordOutput("Shooter/TalonMotor2Temp",m_ShootingMotor2.getDeviceTemp().getValueAsDouble());
     //   Logger.recordOutput("Shooter/CanCoderPositio ",getCanCoderPosition());
     //   Logger.recordOutput("Shooter/RelativePosition",getRelativePosition());
        Logger.recordOutput("Shooter/ShotsTaken",m_shotsTaken);
     //   Logger.recordOutput("Shooter/ShooterRotateSpeed",m_angleEncoder.getVelocity());
      //  SmartDashboard.putBoolean("IsAtPodiumShotAngle",isAtPodiumShot());
      //  SmartDashboard.putBoolean("IsAtMidShotAngle",isAtMidShot());
      //  SmartDashboard.putBoolean("IsAtShortShotAngle",isAtShortShot());
    }
}
