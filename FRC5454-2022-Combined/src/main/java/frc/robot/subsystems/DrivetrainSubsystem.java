package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;
import frc.robot.common.control.*;
import frc.robot.common.drivers.Gyroscope;
import frc.robot.common.drivers.SwerveModule;
import frc.robot.common.kinematics.ChassisVelocity;
import frc.robot.common.kinematics.SwerveKinematics;
import frc.robot.common.kinematics.SwerveOdometry;
import frc.robot.common.math.RigidTransform2;
import frc.robot.common.math.Rotation2;
import frc.robot.common.math.Vector2;
import frc.robot.common.UpdateManager;
import frc.robot.common.drivers.Mk2SwerveModuleBuilder;
import frc.robot.common.drivers.NavX;
import frc.robot.common.util.DrivetrainFeedforwardConstants;
import frc.robot.common.util.HolonomicDriveSignal;
import frc.robot.common.util.InterpolatingDouble;
import frc.robot.common.util.InterpolatingTreeMap;
import frc.robot.common.util.HolonomicFeedforward;

import java.util.Optional;

public class DrivetrainSubsystem implements Subsystem, UpdateManager.Updatable {
    public static final double TRACKWIDTH = 1.0;
    public static final double WHEELBASE = 1.0;

    public static final DrivetrainFeedforwardConstants FEEDFORWARD_CONSTANTS = new DrivetrainFeedforwardConstants(
            0.0584,
            0.00519,
            0.665);

    private static final double GEAR_REDUCTION = 190.0 / 27.0;
    private static final double WHEEL_DIAMETER = 4.0;
    private static final PidConstants MODULE_ANGLE_PID_CONSTANTS = new PidConstants(0.5, 0.0, 0.0001);

    public static final TrajectoryConstraint[] TRAJECTORY_CONSTRAINTS = {
            new FeedforwardConstraint(11.0, FEEDFORWARD_CONSTANTS.getVelocityConstant(),
                    FEEDFORWARD_CONSTANTS.getAccelerationConstant(), false),
            new MaxAccelerationConstraint(6.0 * 12.0),
            new CentripetalAccelerationConstraint(20.0 * 12.0)
    };

    private static final int MAX_LATENCY_COMPENSATION_MAP_ENTRIES = 25;

    private final SwerveModule frontLeftModule = new Mk2SwerveModuleBuilder(
            new Vector2(TRACKWIDTH / 2.0, WHEELBASE / 2.0))
                    .angleMotor(
                            new CANSparkMax(Constants.RobotMap.DRIVETRAIN_FRONT_LEFT_ANGLE_MOTOR, MotorType.kBrushless),
                            MODULE_ANGLE_PID_CONSTANTS)
                    .driveMotor(
                            new CANSparkMax(Constants.RobotMap.DRIVETRAIN_FRONT_LEFT_DRIVE_MOTOR, MotorType.kBrushless),
                            GEAR_REDUCTION,
                            WHEEL_DIAMETER)
                    .angleEncoder(
                            new AnalogInput(Constants.RobotMap.DRIVETRAIN_FRONT_LEFT_ANGLE_ENCODER),
                            Constants.RobotMap.DRIVETRAIN_FRONT_LEFT_ENCODER_OFFSET)
                    .build();

    private final SwerveModule frontRightModule = new Mk2SwerveModuleBuilder(
            new Vector2(TRACKWIDTH / 2.0, -WHEELBASE / 2.0))
                    .angleMotor(
                            new CANSparkMax(Constants.RobotMap.DRIVETRAIN_FRONT_RIGHT_ANGLE_MOTOR,
                                    MotorType.kBrushless),
                            MODULE_ANGLE_PID_CONSTANTS)
                    .driveMotor(
                            new CANSparkMax(Constants.RobotMap.DRIVETRAIN_FRONT_RIGHT_DRIVE_MOTOR,
                                    MotorType.kBrushless),
                            GEAR_REDUCTION,
                            WHEEL_DIAMETER)
                    .angleEncoder(
                            new AnalogInput(Constants.RobotMap.DRIVETRAIN_FRONT_RIGHT_ANGLE_ENCODER),
                            Constants.RobotMap.DRIVETRAIN_FRONT_RIGHT_ENCODER_OFFSET)
                    .build();

    private final SwerveModule backLeftModule = new Mk2SwerveModuleBuilder(
            new Vector2(-TRACKWIDTH / 2.0, WHEELBASE / 2.0))
                    .angleMotor(
                            new CANSparkMax(Constants.RobotMap.DRIVETRAIN_BACK_LEFT_ANGLE_MOTOR, MotorType.kBrushless),
                            MODULE_ANGLE_PID_CONSTANTS)
                    .driveMotor(
                            new CANSparkMax(Constants.RobotMap.DRIVETRAIN_BACK_LEFT_DRIVE_MOTOR, MotorType.kBrushless),
                            GEAR_REDUCTION,
                            WHEEL_DIAMETER)
                    .angleEncoder(
                            new AnalogInput(Constants.RobotMap.DRIVETRAIN_BACK_LEFT_ANGLE_ENCODER),
                            Constants.RobotMap.DRIVETRAIN_BACK_LEFT_ENCODER_OFFSET)
                    .build();

    private final SwerveModule backRightModule = new Mk2SwerveModuleBuilder(
            new Vector2(-TRACKWIDTH / 2.0, -WHEELBASE / 2.0))
                    .angleMotor(
                            new CANSparkMax(Constants.RobotMap.DRIVETRAIN_BACK_RIGHT_ANGLE_MOTOR, MotorType.kBrushless),
                            MODULE_ANGLE_PID_CONSTANTS)
                    .driveMotor(
                            new CANSparkMax(Constants.RobotMap.DRIVETRAIN_BACK_RIGHT_DRIVE_MOTOR, MotorType.kBrushless),
                            GEAR_REDUCTION,
                            WHEEL_DIAMETER)
                    .angleEncoder(
                            new AnalogInput(Constants.RobotMap.DRIVETRAIN_BACK_RIGHT_ANGLE_ENCODER),
                            Constants.RobotMap.DRIVETRAIN_BACK_RIGHT_ENCODER_OFFSET)
                    .build();

    private final SwerveModule[] modules = { frontLeftModule, frontRightModule, backLeftModule, backRightModule };

    private final HolonomicMotionProfiledTrajectoryFollower follower = new HolonomicMotionProfiledTrajectoryFollower(
            new PidConstants(0.4, 0.0, 0.025),
            new PidConstants(5.0, 0.0, 0.0),
            new HolonomicFeedforward(FEEDFORWARD_CONSTANTS));

    private final SwerveKinematics swerveKinematics = new SwerveKinematics(
            new Vector2(TRACKWIDTH / 2.0, WHEELBASE / 2.0), // front left
            new Vector2(TRACKWIDTH / 2.0, -WHEELBASE / 2.0), // front right
            new Vector2(-TRACKWIDTH / 2.0, WHEELBASE / 2.0), // back left
            new Vector2(-TRACKWIDTH / 2.0, -WHEELBASE / 2.0) // back right
    );

    private final Object sensorLock = new Object();

    private Gyroscope gyroscope = new NavX(SPI.Port.kMXP);

    private final Object kinematicsLock = new Object();

    private final SwerveOdometry swerveOdometry = new SwerveOdometry(swerveKinematics, RigidTransform2.ZERO);

    private RigidTransform2 pose = RigidTransform2.ZERO;

    private InterpolatingTreeMap<InterpolatingDouble, RigidTransform2> latencyCompensationMap = new InterpolatingTreeMap<>();

    private Vector2 velocity = Vector2.ZERO;

    private double angularVelocity = 0.0;

    private final Object stateLock = new Object();

    private HolonomicDriveSignal driveSignal = null;

    // Logging
    private final NetworkTableEntry odometryXEntry;
    private final NetworkTableEntry odometryYEntry;
    private final NetworkTableEntry odometryAngleEntry;

    private final NetworkTableEntry[] moduleAngleEntries = new NetworkTableEntry[modules.length];

    public DrivetrainSubsystem() {
        synchronized (sensorLock) {
            gyroscope.setInverted(false);
        }

        ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");
        odometryXEntry = tab.add("X", 0.0)
                .withPosition(0, 0)
                .withSize(1, 1)
                .getEntry();
        odometryYEntry = tab.add("Y", 0.0)
                .withPosition(0, 1)
                .withSize(1, 1)
                .getEntry();
        odometryAngleEntry = tab.add("Angle", 0.0)
                .withPosition(0, 2)
                .withSize(1, 1)
                .getEntry();
        tab.addNumber("Trajectory X", () -> {
            if (follower.getLastState() == null) {
                return 0.0;
            }
            return follower.getLastState().getPathState().getPosition().x;
        })
                .withPosition(1, 0)
                .withSize(1, 1);
        tab.addNumber("Trajectory Y", () -> {
            if (follower.getLastState() == null) {
                return 0.0;
            }
            return follower.getLastState().getPathState().getPosition().y;
        })
                .withPosition(1, 1)
                .withSize(1, 1);

        ShuffleboardLayout[] moduleLayouts = {
                tab.getLayout("Front Left Module", BuiltInLayouts.kList),
                tab.getLayout("Front Right Module", BuiltInLayouts.kList),
                tab.getLayout("Back Left Module", BuiltInLayouts.kList),
                tab.getLayout("Back Right Module", BuiltInLayouts.kList)
        };
        for (int i = 0; i < modules.length; i++) {
            ShuffleboardLayout layout = moduleLayouts[i]
                    .withPosition(2 + i * 2, 0)
                    .withSize(2, 4);
            moduleAngleEntries[i] = layout.add("Angle", 0.0).getEntry();
        }
        tab.addNumber("Rotation Voltage", () -> {
            HolonomicDriveSignal signal;
            synchronized (stateLock) {
                signal = driveSignal;
            }

            if (signal == null) {
                return 0.0;
            }

            return signal.getRotation() * RobotController.getBatteryVoltage();
        });
    }

    public RigidTransform2 getPose() {
        synchronized (kinematicsLock) {
            return pose;
        }
    }

    public Vector2 getVelocity() {
        synchronized (kinematicsLock) {
            return velocity;
        }
    }

    public double getAngularVelocity() {
        synchronized (kinematicsLock) {
            return angularVelocity;
        }
    }

    public void drive(Vector2 translationalVelocity, double rotationalVelocity, boolean isFieldOriented) {
        synchronized (stateLock) {
            driveSignal = new HolonomicDriveSignal(translationalVelocity, rotationalVelocity, isFieldOriented);
        }
    }

    public void move (double direction, double speed, double distance, boolean stopAtEnd)
    {       double startDistance;
            double forward=0;
            double strafe=0;
            double rotation=0;
            startDistance=backLeftModule.getCurrentDistance();
            switch ((int) direction){
                    case 0:
                            forward=1*speed;
                            strafe=0;
                            break;
                    case 45:
                            forward=1*speed;
                            strafe=1*speed;
                            break;
                   
                    case 90:
                            forward=0;
                            strafe=1*speed;
                            break;
                    case 135:
                            forward=-1*speed;
                            strafe=1*speed;
                            break;
                    case 180:
                            forward=-1*speed;
                            strafe=0;
                            break;
                    case 225:
                            forward=-1*speed;
                            strafe=-1*speed;
                            break;
                    case 270:
                            forward=0;
                            strafe=-1*speed;
                            break;
                    case 315:
                            forward=1*speed;
                            strafe=-1*speed;
                            break;
                    
            }
            double distanceTravelled=backLeftModule.getCurrentDistance()-startDistance;
            do {
                  drive(new Vector2(forward, strafe), rotation, true);
                  periodic();
                  distanceTravelled=Math.abs(backLeftModule.getCurrentDistance()-startDistance);
                  System.out.print("(" + forward + ", "+ strafe +") " + distanceTravelled + " / " + distance );
            } while(distanceTravelled<=distance);
            if (stopAtEnd) {
                    drive(new Vector2(0,0), 0, true);
                    periodic();                
            }
    }

    public void resetGyroscope() {
        gyroscope.setAdjustmentAngle(gyroscope.getUnadjustedAngle());
    }

    public void resetPose(RigidTransform2 pose) {
        synchronized (kinematicsLock) {
            this.pose = pose;
            swerveOdometry.resetPose(pose);
        }
    }

    public void resetGyroAngle(Rotation2 angle) {
        synchronized (sensorLock) {
            gyroscope.setAdjustmentAngle(
                    gyroscope.getUnadjustedAngle().rotateBy(angle.inverse()));
        }
    }

    private void updateOdometry(double time, double dt) {
        Vector2[] moduleVelocities = new Vector2[modules.length];
        for (int i = 0; i < modules.length; i++) {
            var module = modules[i];
            module.updateSensors();

            moduleVelocities[i] = Vector2.fromAngle(Rotation2.fromRadians(module.getCurrentAngle()))
                    .scale(module.getCurrentVelocity());
        }

        Rotation2 angle;
        double angularVelocity;
        synchronized (sensorLock) {
            angle = gyroscope.getAngle();
            angularVelocity = gyroscope.getRate();
        }

        ChassisVelocity velocity = swerveKinematics.toChassisVelocity(moduleVelocities);

        synchronized (kinematicsLock) {
            this.pose = swerveOdometry.update(angle, dt, moduleVelocities);
            if (latencyCompensationMap.size() > MAX_LATENCY_COMPENSATION_MAP_ENTRIES) {
                latencyCompensationMap.remove(latencyCompensationMap.firstKey());
            }
            latencyCompensationMap.put(new InterpolatingDouble(time), pose);
            this.velocity = velocity.getTranslationalVelocity();
            this.angularVelocity = angularVelocity;
        }
    }

    private void updateModules(HolonomicDriveSignal driveSignal, double dt) {
        ChassisVelocity chassisVelocity;
        if (driveSignal == null) {
            chassisVelocity = new ChassisVelocity(Vector2.ZERO, 0.0);
        } else if (driveSignal.isFieldOriented()) {
            chassisVelocity = new ChassisVelocity(
                    driveSignal.getTranslation().rotateBy(getPose().rotation.inverse()),
                    driveSignal.getRotation());
        } else {
            chassisVelocity = new ChassisVelocity(
                    driveSignal.getTranslation(),
                    driveSignal.getRotation());
        }

        Vector2[] moduleOutputs = swerveKinematics.toModuleVelocities(chassisVelocity);
        SwerveKinematics.normalizeModuleVelocities(moduleOutputs, 1);
        for (int i = 0; i < moduleOutputs.length; i++) {
            var module = modules[i];
            module.setTargetVelocity(moduleOutputs[i]);
            module.updateState(dt);
        }
    }

    public RigidTransform2 getPoseAtTime(double timestamp) {
        synchronized (kinematicsLock) {
            if (latencyCompensationMap.isEmpty()) {
                return RigidTransform2.ZERO;
            }
            return latencyCompensationMap.getInterpolated(new InterpolatingDouble(timestamp));
        }
    }

    @Override
    public void update(double time, double dt) {
        updateOdometry(time, dt);

        HolonomicDriveSignal driveSignal;
        Optional<HolonomicDriveSignal> trajectorySignal = follower.update(
                getPose(),
                getVelocity(),
                getAngularVelocity(),
                time,
                dt);
        if (trajectorySignal.isPresent()) {
            driveSignal = trajectorySignal.get();
            driveSignal = new HolonomicDriveSignal(
                    driveSignal.getTranslation().scale(1.0 / RobotController.getBatteryVoltage()),
                    driveSignal.getRotation() / RobotController.getBatteryVoltage(),
                    driveSignal.isFieldOriented());
        } else {
            synchronized (stateLock) {
                driveSignal = this.driveSignal;
            }
        }

        updateModules(driveSignal, dt);
    }

    @Override
    public void periodic() {
        RigidTransform2 pose = getPose();
        odometryXEntry.setDouble(pose.translation.x);
        odometryYEntry.setDouble(pose.translation.y);
        odometryAngleEntry.setDouble(getPose().rotation.toDegrees());

        for (int i = 0; i < modules.length; i++) {
            moduleAngleEntries[i].setDouble(Math.toDegrees(modules[i].getCurrentAngle()));
        }
    }

    public TrajectoryFollower<HolonomicDriveSignal> getFollower() {
        return follower;
    }
}
