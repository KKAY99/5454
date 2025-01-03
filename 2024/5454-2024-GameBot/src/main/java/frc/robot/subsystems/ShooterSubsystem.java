package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.io.Console;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderStatusFrame;
import com.ctre.phoenix.sensors.WPI_CANCoder;
import com.revrobotics.SparkMaxPIDController;
import frc.robot.utilities.ObsidianCANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;

import frc.robot.Constants;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.utilities.Limelight;
import frc.robot.utilities.ShotTable;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.BaseStatusSignal;

public class ShooterSubsystem extends SubsystemBase {
    private Limelight m_limeLight;

    private TalonFX m_ShootingMotor1;
    private TalonFX m_ShootingMotor2;
    private ObsidianCANSparkMax m_feederMotor;
    private ObsidianCANSparkMax m_angleMotor;

    private SparkMaxPIDController m_anglePID;

    private RelativeEncoder m_angleEncoder;
    private WPI_CANCoder m_canCoder;

    private double m_baseSpeed;
    private double m_distance;

    private double m_targetAngle = 0;
    private double m_shotsTaken;
    private double m_desiredVeloc1;
    private double m_desiredVeloc2;
    private Orchestra m_orchestra = new Orchestra();
    public ShooterSubsystem(Limelight limeLight, int shootingMotor1, int shootingMotor2, int angleMotor, int feedMotor,
            int canCoderId, double baseSpeed) {
        m_limeLight = limeLight;
        m_baseSpeed = baseSpeed;
        m_ShootingMotor1 = new TalonFX(shootingMotor1);
        configmotor(m_ShootingMotor1);
        m_ShootingMotor2 = new TalonFX(shootingMotor2);
        configmotor(m_ShootingMotor2);
        m_feederMotor =new ObsidianCANSparkMax(feedMotor,MotorType.kBrushless,false,Constants.k30Amp);
        m_angleMotor = new ObsidianCANSparkMax(angleMotor, MotorType.kBrushless,true,Constants.k40Amp);
 
        m_anglePID = m_angleMotor.getPIDController();
        m_canCoder = new WPI_CANCoder(canCoderId);
        m_canCoder.setStatusFramePeriod(CANCoderStatusFrame.VbatAndFaults, 1000);
                
        m_angleEncoder = m_angleMotor.getEncoder();

        double anglekP = 0.3;// 0.8
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
        m_angleMotor.burnFlash();
    }

    public void configmotor(TalonFX motor) {

        // fetch *all* configs currently applied to the device
        var slot0Configs = new Slot0Configs();
        slot0Configs.kV = 0.12;
        slot0Configs.kP = 0.11;
        slot0Configs.kI = 0.48;
        slot0Configs.kD = 0.01;
        BaseStatusSignal.setUpdateFrequencyForAll(
                50,
                motor.getPosition(),
                motor.getVelocity(),
                motor.getMotorVoltage(),
                motor.getRotorVelocity());
        motor.optimizeBusUtilization();
        // apply gains, 50 ms totl timeout
        motor.getConfigurator().apply(slot0Configs, 0.050);
        // var fx_cfg = new TalonFXConfiguration();

        /**
         * Configure the current limits that will be used
         * Stator Current is the current that passes through the motor stators.
         * Use stator current limits to limit rotor acceleration/heat production
         * Supply Current is the current that passes into the controller from the
         * supply.
         * Use supply current limits to prevent breakers from tripping
         *
         * 
         * //will need to use enabled | Limit(amp) | Trigger Threshold(amp) | Trigger
         * Threshold Time(s)
         */
        // var fx_cfg = new TalonFXConfiguration();
        // fx_cfg.CurrentLimits.SupplyCurrentLimit=60;
        // fx_cfg.CurrentLimits.SupplyCurrentLimitEnable=true;
        // fx_cfg.CurrentLimits.SupplyCurrentThreshold=0.5;
        // motor.getConfigurator().apply(fx_cfg);
        // motor.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 20,
        // 25, 1.0));
        // motor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 10,
        // 15, 0.5));

    }
    public void PrimeShootingMotors(){
        double targetSpeed=Constants.ShooterConstants.primeMotorSpeed;
        VelocityVoltage m_velocity = new VelocityVoltage(0);
        m_ShootingMotor1.setControl(m_velocity.withVelocity(targetSpeed));
        m_ShootingMotor2.setControl(m_velocity.withVelocity(targetSpeed));
    }

    public void RunShootingMotors(double veloc1, double veloc2, boolean runFeedMotors) {
        // values are for the indivudal shooter motors
        m_desiredVeloc1 = veloc1;
        m_desiredVeloc2 = veloc2;

        VelocityVoltage m_velocity = new VelocityVoltage(0);
        m_velocity.Slot = 0;

        m_ShootingMotor1.setControl(m_velocity.withVelocity(veloc1));
        m_ShootingMotor2.setControl(m_velocity.withVelocity(veloc2));

        if (runFeedMotors) {
            if (veloc1 > 0) {
                m_feederMotor.set(-ShooterConstants.feederSpeed);
            } else {
                m_feederMotor.set(ShooterConstants.feederSpeed);
            }
        }
    }

    public void RunShootingMotors(double veloc1, double veloc2, boolean runFeedMotors, double feederSpeed) {
        // values are for the indivudal shooter motors
        m_desiredVeloc1 = veloc1;
        m_desiredVeloc2 = veloc2;

        VelocityVoltage m_velocity = new VelocityVoltage(0);
        m_velocity.Slot = 0;

        m_ShootingMotor1.setControl(m_velocity.withVelocity(veloc1));
        m_ShootingMotor2.setControl(m_velocity.withVelocity(veloc2));

        if (runFeedMotors) {
            if (veloc1 > 0) {
                m_feederMotor.set(-feederSpeed);
            } else {
                m_feederMotor.set(feederSpeed);
            }
        }
    }

    public void RunFeedRollers(double speed) {
        m_feederMotor.set(speed);
    }

    public void StopFeedRollers() {
        m_feederMotor.set(0);
    }

    public void SlowShootingMotors() {
        // keep motors running
        VelocityVoltage m_velocity = new VelocityVoltage(0);
        m_velocity.Slot = 0;
        m_ShootingMotor1.setControl(m_velocity.withVelocity(m_baseSpeed));
        m_ShootingMotor2.setControl(m_velocity.withVelocity(m_baseSpeed));

        m_feederMotor.set(0);
    }

    public void stopShooter() {
        m_ShootingMotor1.set(0);
        m_ShootingMotor2.set(0);
        m_feederMotor.set(0);

    }

    public boolean isMotorVelocitysAtDesiredSpeed(double veloc1, double veloc2) {
        boolean returnValue = false;
        boolean bMotor1UptoSpeed = false;
        boolean bMotor2UptoSpeed = false;

        // System.out.println("Veloc1 "+veloc1);
        // System.out.println("CurrentVeloc "+ GetVelocityMotor1());

        bMotor1UptoSpeed = Math.abs(GetVelocityMotor1()) + Math.abs(ShooterConstants.baseSpeedDeadband) >= Math
                .abs(veloc1);
        bMotor2UptoSpeed = Math.abs(GetVelocityMotor2()) + Math.abs(ShooterConstants.baseSpeedDeadband) >= Math
                .abs(veloc2);

        if (bMotor1UptoSpeed && bMotor2UptoSpeed) {

            returnValue = true;
        }

        return returnValue;
    }

    public double GetVelocityMotor1() {
       // System.out.println(m_ShootingMotor1.getVelocity() + " = " + m_ShootingMotor1.getRotorVelocity());
        return m_ShootingMotor1.getRotorVelocity().getValueAsDouble();

    }

    public double GetVelocityMotor2() {
        return m_ShootingMotor2.getRotorVelocity().getValueAsDouble();
    }

    public void OutPutDistance() {
        ShotTable shotTable = new ShotTable();
        double calculation = shotTable.getVelocity1(m_limeLight.getDistance());
        System.out.println("Distance Calucations: " + calculation);
    }

    public void RotateShooter(double speed) {
        ResetControlType();
        m_angleMotor.set(speed);
    }

    public void stopRotate() {
        m_anglePID.setReference(0, ControlType.kVelocity);
        m_angleMotor.set(0);
    }

    public void setAngle(double targetAngle) {
        m_targetAngle = targetAngle;
        
        if(!isSetAnglePastRotateLimit(targetAngle)){
            m_anglePID.setReference(targetAngle, ControlType.kPosition);
        } else {
            System.out.println("Set Angle out of Range");
        }
          
    }

    public double getCanCoderPosition() {
        return m_canCoder.getAbsolutePosition();
    }

    public void zeroRelativePosition() {
        m_angleEncoder.setPosition(0);
    }

    public void zeroCanCoderPosition() {
        m_canCoder.setPosition(0);
    }

    public double getRelativePosition() {
        return m_angleEncoder.getPosition();
    }

    public void ResetControlType() {
        m_anglePID.setReference(0, ControlType.kVelocity);
    }

    public void setBrakeOn() {
        m_ShootingMotor1.setNeutralMode(NeutralModeValue.Brake);
        m_ShootingMotor1.setNeutralMode(NeutralModeValue.Brake);
        // m_angleMotor.setIdleMode(IdleMode.kBrake);
    }

    public void setCoastOn() {
        m_ShootingMotor1.setNeutralMode(NeutralModeValue.Coast);
        m_ShootingMotor1.setNeutralMode(NeutralModeValue.Coast);
        // m_angleMotor.setIdleMode(IdleMode.kCoast);
    }

    public void ResetShotsTaken() {
        m_shotsTaken = 0;
    }

    public void ShotTaken() {
        m_shotsTaken += 1;
    }

    public void PutTableMultiplier() {
    }

    public boolean isAtPodiumShot() {
        boolean returnValue = false;

        if (getRelativePosition() > Math.abs(Constants.ShooterConstants.podiumShotEncoderVal)
                - Constants.ShooterConstants.shooterPosDeadband &&
                getRelativePosition() < Math.abs(Constants.ShooterConstants.podiumShotEncoderVal)
                        + Constants.ShooterConstants.shooterPosDeadband) {
            returnValue = true;
        }

        return returnValue;
    }

    public boolean isAtHomeValue(){
        if(getCanCoderPosition()<=Constants.ShooterConstants.homePos+Constants.ShooterConstants.homeLEDCheckDeadband
          &&  getCanCoderPosition()>=Constants.ShooterConstants.homePos-Constants.ShooterConstants.homeLEDCheckDeadband){    
    
    //         System.out.println("Is At Home Value");
              return true;
     } else {
     //   System.out.println("Is Not At Home Value " + getCanCoderPosition());
        return false;
     }
    }

    public boolean isAtStow(){
        return (getRelativePosition()-0.3<ShooterConstants.shooterStowAngle&&getRelativePosition()+0.3>ShooterConstants.shooterStowAngle);
    }

    public boolean isAtMidShot() {
        boolean returnValue = false;

        if (getRelativePosition() > Math.abs(Constants.ShooterConstants.midShotEncoderVal)
                - Constants.ShooterConstants.shooterPosDeadband &&
                getRelativePosition() < Math.abs(Constants.ShooterConstants.midShotEncoderVal)
                        + Constants.ShooterConstants.shooterPosDeadband) {
            returnValue = true;
        }

        return returnValue;
    }

    public boolean isAtShortShot() {
        boolean returnValue = false;

        if (getRelativePosition() > Math.abs(Constants.ShooterConstants.shortShotEncoderVal)
                - Constants.ShooterConstants.shooterPosDeadband &&
                getRelativePosition() < Math.abs(Constants.ShooterConstants.shortShotEncoderVal)
                        + Constants.ShooterConstants.shooterPosDeadband) {
            returnValue = true;
        }

        return returnValue;
    }

    public boolean hasHitRotateLimit(double speed) {
        boolean returnValue = false;
        if (speed > 0) {
            // if going up in speed and hitting HighSoftLimit then return true
            returnValue = (getRelativePosition() >= Constants.ShooterConstants.rotateHighSoftLimit);
        } else {
            // negatives speed so check low soft limit
            returnValue = (getRelativePosition() <= Constants.ShooterConstants.rotateLowSoftLimit);
        }
        return returnValue;
    }

    public boolean isSetAnglePastRotateLimit(double angle) {
        boolean returnValue = false;
        
        if(angle>ShooterConstants.rotateHighSoftLimit||angle<ShooterConstants.rotateLowSoftLimit){
            returnValue=true;
        }
        return returnValue;
    }

    public void setupMusic(){
        Orchestra orchestra = new Orchestra();
        // Add  devices to the orchestra
        m_orchestra.addInstrument(m_ShootingMotor1);
        m_orchestra.addInstrument(m_ShootingMotor2);
   
    }
    public void playMusic(){
        // Add  devices to the orchestra
        m_orchestra.addInstrument(m_ShootingMotor1);
        m_orchestra.addInstrument(m_ShootingMotor2);
        // Attempt to load the chrp
        var status = m_orchestra.loadMusic("ImperialTuner.chrp");
        if (status.isOK()) {
            System.out.println("Playing Music");
            m_orchestra.play();
            
        }
        else{
            System.out.println("Music File Load error " + status.getDescription());
        }
    }
       
    @Override
    public void periodic() {
        Logger.recordOutput("Shooter/Shooter1VelocitySet", m_desiredVeloc1);
        Logger.recordOutput("Shooter/Shooter2VelocitySet", m_desiredVeloc2);
        Logger.recordOutput("Shooter/ShooterSetAngle", m_targetAngle);
        Logger.recordOutput("Shooter/ShootMotor1Velocity",GetVelocityMotor1());
        Logger.recordOutput("Shooter/ShootMotor2Velocity",GetVelocityMotor2());
        // Logger.recordOutput("Shooter/TalonMotor1Temp",m_ShootingMotor1.getDeviceTemp().getValueAsDouble());
        // Logger.recordOutput("Shooter/TalonMotor2Temp",m_ShootingMotor2.getDeviceTemp().getValueAsDouble());
        Logger.recordOutput("Shooter/CanCoderPositio ",getCanCoderPosition());
        Logger.recordOutput("Shooter/RelativePosition",getRelativePosition());
        Logger.recordOutput("Shooter/ShotsTaken", m_shotsTaken);
        Logger.recordOutput("Shooter/ShooterRotateSpeed",m_angleEncoder.getVelocity());
        // SmartDashboard.putBoolean("IsAtPodiumShotAngle",isAtPodiumShot());
        // SmartDashboard.putBoolean("IsAtMidShotAngle",isAtMidShot());
        // SmartDashboard.putBoolean("IsAtShortShotAngle",isAtShortShot());
    }
}
