package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.Constants;
import frc.robot.Constants.HoodConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.shooter.TurretUtil.ShotSolution;
import frc.robot.subsystems.shooter.TurretUtil.TargetType;
import frc.robot.utilities.ObsidianCANSparkMax;

public class NewShooterSubsystem extends SubsystemBase {

    // -------------------------
    // Hardware
    // -------------------------
    private final TalonFX m_1shooterMotor;
    private final TalonFX m_2shooterMotor;
    private final TalonFX m_hoodMotor;
    private final CANcoder m_hoodCoder;
    private final ObsidianCANSparkMax m_kickerMotor;

    // -------------------------
    // Drivetrain reference (for pose)
    // -------------------------
    private final CommandSwerveDrivetrain drivetrain;

    // -------------------------
    // State
    // -------------------------
    private ShotSolution lastShotSolution = null;

    // -------------------------
    // Constructor
    // -------------------------
    public NewShooterSubsystem(int shooter1CANID, int shooter2CANID,
                                int kickerCANID, int hoodCANID,
                                CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;

        m_hoodCoder = new CANcoder(HoodConstants.hoodCoderCANID);
        m_1shooterMotor = new TalonFX(shooter1CANID);
        configureShooterMotor(m_1shooterMotor);
        m_1shooterMotor.setNeutralMode(NeutralModeValue.Coast);

        m_2shooterMotor = new TalonFX(shooter2CANID);
        configureShooterMotor(m_2shooterMotor);
        m_2shooterMotor.setNeutralMode(NeutralModeValue.Coast);

        m_hoodMotor = new TalonFX(hoodCANID);
        m_hoodMotor.setNeutralMode(NeutralModeValue.Brake);

        m_kickerMotor = new ObsidianCANSparkMax(kickerCANID, MotorType.kBrushless, false, Constants.k70Amp);

        CANcoderConfiguration hoodCoder_cfg = new CANcoderConfiguration();
        hoodCoder_cfg.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1.0;
        hoodCoder_cfg.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        hoodCoder_cfg.MagnetSensor.withMagnetOffset(Rotations.of(HoodConstants.hoodOffset));
        m_hoodCoder.getConfigurator().apply(hoodCoder_cfg);
    }

    // -------------------------
    // Shot from pose (TurretUtil)
    // -------------------------

    /**
     * Computes the shot solution from the current robot pose and spins up
     * the shooter and hood to the correct values. Does NOT fire the kicker.
     * Call runKicker() separately when atTargetSpeed() returns true.
     */
    public void spinUpFromPose(TargetType target) {
        lastShotSolution = TurretUtil.computeShotSolution(
                drivetrain.getState().Pose, target);

        if (lastShotSolution.isValid) {
            runShooterVelocity(lastShotSolution.shooterSpeedRPS);
            poormanHoldHoodPos(
                    lastShotSolution.trajectoryAngleDegrees,
                    HoodConstants.hoodSpeed,
                    HoodConstants.hoodDeadband);
        }
    }

    /** Returns true when the flywheel is within deadband of the last computed target speed. */
    public boolean readyToShoot() {
        if (lastShotSolution == null || !lastShotSolution.isValid) return false;
        return atTargetSpeed(lastShotSolution.shooterSpeedRPS);
    }

    // -------------------------
    // Hood
    // -------------------------
    public double getHoodPos() {
        double currentPos = m_hoodCoder.getAbsolutePosition().getValueAsDouble();
        return (currentPos < 0.95) ? currentPos : 0.0;
    }

    public boolean checkHoodPos(double hoodTarget, double hoodSpeed, double deadband) {
        return Math.abs(getHoodPos() - hoodTarget) <= deadband;
    }

    public void poormanHoldHoodPos(double hoodTarget, double hoodSpeed, double deadband) {
        double currentPos = getHoodPos();
        double hoodDiff = Math.abs(currentPos - hoodTarget);
        if (hoodDiff > deadband) {
            m_hoodMotor.set(currentPos > hoodTarget ? -hoodSpeed : hoodSpeed);
        } else {
            m_hoodMotor.stopMotor();
        }
    }

    public void hoodMoveToZero() {
        double hoodTarget = 0;
        double hoodSpeed = HoodConstants.hoodSpeed;
        double deadband = HoodConstants.hoodDeadband;
        while (!checkHoodPos(hoodTarget, hoodSpeed, deadband)) {
            poormanHoldHoodPos(hoodTarget, hoodSpeed, deadband);
        }
        stopHood();
    }

    public void hoodMove(double hoodSpeed) {
        m_hoodMotor.set(hoodSpeed);
    }

    public void stopHood() {
        m_hoodMotor.stopMotor();
    }

    // -------------------------
    // Shooter / Kicker
    // -------------------------
    public void runShooterVelocity(double targetSpeed) {
        m_1shooterMotor.setControl(new VelocityVoltage(targetSpeed));
        m_2shooterMotor.setControl(new VelocityVoltage(-targetSpeed));
    }

    public void runKicker(double kickerSpeed) {
        m_kickerMotor.set(kickerSpeed);
    }

    public void runNewShooter(double speed, double kickerSpeed) {
        runShooterVelocity(speed);
        m_kickerMotor.set(kickerSpeed);
    }

    public boolean atTargetSpeed(double targetSpeed) {
        double currentSpeed = m_1shooterMotor.getVelocity().getValueAsDouble();
        double speedDiff = Math.abs(currentSpeed - targetSpeed);
        return (currentSpeed > targetSpeed)
                ? speedDiff < ShooterConstants.shooterVelocityHighDeadband
                : speedDiff < ShooterConstants.shooterVelocityLowDeadband;
    }

    public void stopNewShooter(boolean idleMode) {
        m_1shooterMotor.stopMotor();
        m_2shooterMotor.stopMotor();
        m_kickerMotor.stopMotor();
        hoodMoveToZero();
    }

    // -------------------------
    // Commands
    // -------------------------

    /**
     * Spins up shooter and hood from pose, fires kicker when at speed.
     * Stops everything on end.
     */
    public Command shootFromPoseCommand(TargetType target) {
        return Commands.run(() -> {
            spinUpFromPose(target);
            if (readyToShoot()) {
                runKicker(ShooterConstants.KickerSpeed);
            }
        }, this).finallyDo(interrupted -> stopNewShooter(true));
    }

    /** Convenience wrapper for the common HUB shot. */
    public Command shootHubCommand() {
        return shootFromPoseCommand(TargetType.HUB);
    }

    /** Legacy fixed-speed shoot command — kept for manual override. */
    public Command shootCommand() {
        return Commands.startEnd(
                () -> runNewShooter(ShooterConstants.shootSpeed, ShooterConstants.KickerSpeed),
                () -> stopNewShooter(true),
                this);
    }

    public Command shootOnCommand() {
        return Commands.runOnce(
                () -> runNewShooter(ShooterConstants.shootSpeed, ShooterConstants.KickerSpeed), this);
    }

    public Command shootOffCommand() {
        return Commands.runOnce(() -> stopNewShooter(true), this);
    }

    public Command shutdownCommand() {
        return Commands.runOnce(() -> stopNewShooter(false), this);
    }

    public Command hoodHomeCommand() {
        return Commands.runOnce(this::hoodMoveToZero, this);
    }

    public Command hoodUpCommand() {
        return Commands.startEnd(
                () -> hoodMove(HoodConstants.hoodUpSpeed),
                this::stopHood,
                this);
    }

    public Command hoodDownCommand() {
        return Commands.startEnd(
                () -> hoodMove(HoodConstants.hoodDownSpeed),
                this::stopHood,
                this);
    }

    // -------------------------
    // Periodic
    // -------------------------
    @Override
    public void periodic() {
        SmartDashboard.putNumber("Shooter Velocity", m_1shooterMotor.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber("Hood CanCoder Value", m_hoodCoder.getAbsolutePosition().getValueAsDouble());
        SmartDashboard.putNumber("Hood Pos", getHoodPos());

        // Live shot solution telemetry
        ShotSolution shot = TurretUtil.computeShotSolution(
                drivetrain.getState().Pose, TargetType.HUB);
        SmartDashboard.putNumber("Distance to Hub (m)", shot.distanceMeters);
        SmartDashboard.putNumber("Target Shooter RPS", shot.shooterSpeedRPS);
        SmartDashboard.putNumber("Target Hood Angle", shot.trajectoryAngleDegrees);
        SmartDashboard.putNumber("Turret Angle Required", shot.turretAngleDegrees);
        SmartDashboard.putBoolean("Shot Valid", shot.isValid);
        SmartDashboard.putBoolean("Ready to Shoot", readyToShoot());
    }

    // -------------------------
    // Motor config (private)
    // -------------------------
    private void configureShooterMotor(TalonFX motor) {
        TalonFXConfigurator configurator = motor.getConfigurator();
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.Slot0.kP = 999999.0;
        config.TorqueCurrent.PeakForwardTorqueCurrent = 800.0;
        config.TorqueCurrent.PeakReverseTorqueCurrent = -800.0;
        config.MotorOutput.PeakForwardDutyCycle = 1.0;
        config.MotorOutput.PeakReverseDutyCycle = -1.0;
        configurator.apply(config);

        CurrentLimitsConfigs currentLimits = new CurrentLimitsConfigs();
        currentLimits.StatorCurrentLimit = 80;
        currentLimits.SupplyCurrentLimit = 60;
        configurator.apply(currentLimits);
    }
}
