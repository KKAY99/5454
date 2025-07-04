package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;
import java.util.function.Supplier;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.PointWheelsAt;
import com.ctre.phoenix6.swerve.SwerveRequest.SwerveDriveBrake;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.DriveFeedforwards;
import frc.robot.Constants;
import frc.robot.TunerConstants;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.TunerConstants.TunerSwerveDrivetrain;
import org.littletonrobotics.junction.Logger;

/**
 * Class that extends the Phoenix 6 SwerveDrivetrain class and implements
 * Subsystem so it can easily be used in command-based projects.
 */
public class CommandSwerveDrivetrain extends TunerSwerveDrivetrain implements Subsystem {
    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;

    /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
    private static final Rotation2d kBlueAlliancePerspectiveRotation = Rotation2d.kZero;
    /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
    private static final Rotation2d kRedAlliancePerspectiveRotation = Rotation2d.k180deg;
    /* Keep track if we've ever applied the operator perspective before or not */
    private boolean m_hasAppliedOperatorPerspective = false;

    /* Swerve requests to apply during SysId characterization */
    private final SwerveRequest.SysIdSwerveTranslation m_translationCharacterization = new SwerveRequest.SysIdSwerveTranslation();
    private final SwerveRequest.SysIdSwerveSteerGains m_steerCharacterization = new SwerveRequest.SysIdSwerveSteerGains();
    private final SwerveRequest.SysIdSwerveRotation m_rotationCharacterization = new SwerveRequest.SysIdSwerveRotation();

    private SwerveRequest.ApplyRobotSpeeds autoDrive = new SwerveRequest.ApplyRobotSpeeds();

    private SwerveDrivePoseEstimator m_poseEstimator;

    private double m_gasPedalDriveMult=1;
    private double m_gasPedalRotMult=1;

    /* SysId routine for characterizing translation. This is used to find PID gains for the drive motors. */
    private final SysIdRoutine m_sysIdRoutineTranslation = new SysIdRoutine(
        new SysIdRoutine.Config(
            null,        // Use default ramp rate (1 V/s)
            Volts.of(4), // Reduce dynamic step voltage to 4 V to prevent brownout
            null,        // Use default timeout (10 s)
            // Log state with SignalLogger class
            state -> SignalLogger.writeString("SysIdTranslation_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            output -> setControl(m_translationCharacterization.withVolts(output)),
            null,
            this
        )
    );

    /* SysId routine for characterizing steer. This is used to find PID gains for the steer motors. */
    private final SysIdRoutine m_sysIdRoutineSteer = new SysIdRoutine(
        new SysIdRoutine.Config(
            null,        // Use default ramp rate (1 V/s)
            Volts.of(7), // Use dynamic voltage of 7 V
            null,        // Use default timeout (10 s)
            // Log state with SignalLogger class
            state -> SignalLogger.writeString("SysIdSteer_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            volts -> setControl(m_steerCharacterization.withVolts(volts)),
            null,
            this
        )
    );

    /*
     * SysId routine for characterizing rotation.
     * This is used to find PID gains for the FieldCentricFacingAngle HeadingController.
     * See the documentation of SwerveRequest.SysIdSwerveRotation for info on importing the log to SysId.
     */
    private final SysIdRoutine m_sysIdRoutineRotation = new SysIdRoutine(
        new SysIdRoutine.Config(
            /* This is in radians per second², but SysId only supports "volts per second" */
            Volts.of(Math.PI / 6).per(Second),
            /* This is in radians per second, but SysId only supports "volts" */
            Volts.of(Math.PI),
            null, // Use default timeout (10 s)
            // Log state with SignalLogger class
            state -> SignalLogger.writeString("SysIdRotation_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            output -> {
                /* output is actually radians per second, but SysId only supports "volts" */
                setControl(m_rotationCharacterization.withRotationalRate(output.in(Volts)));
                /* also log the requested output for SysId */
                SignalLogger.writeDouble("Rotational_Rate", output.in(Volts));
            },
            null,
            this
        )
    );

    /* The SysId routine to test */
    private SysIdRoutine m_sysIdRoutineToApply = m_sysIdRoutineTranslation;

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not construct
     * the devices themselves. If they need the devices, they can access them through
     * getters in the classes.
     *
     * @param drivetrainConstants   Drivetrain-wide constants for the swerve drive
     * @param modules               Constants for each specific module
     */
    public CommandSwerveDrivetrain(
        SwerveDrivetrainConstants drivetrainConstants,
        SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(drivetrainConstants, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }

        buildPoseEstimator();
        configAutoBuilder();
    }

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not construct
     * the devices themselves. If they need the devices, they can access them through
     * getters in the classes.
     *
     * @param drivetrainConstants     Drivetrain-wide constants for the swerve drive
     * @param odometryUpdateFrequency The frequency to run the odometry loop. If
     *                                unspecified or set to 0 Hz, this is 250 Hz on
     *                                CAN FD, and 100 Hz on CAN 2.0.
     * @param modules                 Constants for each specific module
     */
    public CommandSwerveDrivetrain(
        SwerveDrivetrainConstants drivetrainConstants,
        double odometryUpdateFrequency,
        SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(drivetrainConstants, odometryUpdateFrequency, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }

        buildPoseEstimator();
        configAutoBuilder();
    }

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not construct
     * the devices themselves. If they need the devices, they can access them through
     * getters in the classes.
     *
     * @param drivetrainConstants       Drivetrain-wide constants for the swerve drive
     * @param odometryUpdateFrequency   The frequency to run the odometry loop. If
     *                                  unspecified or set to 0 Hz, this is 250 Hz on
     *                                  CAN FD, and 100 Hz on CAN 2.0.
     * @param odometryStandardDeviation The standard deviation for odometry calculation
     *                                  in the form [x, y, theta]ᵀ, with units in meters
     *                                  and radians
     * @param visionStandardDeviation   The standard deviation for vision calculation
     *                                  in the form [x, y, theta]ᵀ, with units in meters
     *                                  and radians
     * @param modules                   Constants for each specific module
     */
    public CommandSwerveDrivetrain(
        SwerveDrivetrainConstants drivetrainConstants,
        double odometryUpdateFrequency,
        Matrix<N3, N1> odometryStandardDeviation,
        Matrix<N3, N1> visionStandardDeviation,
        SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(drivetrainConstants, odometryUpdateFrequency, odometryStandardDeviation, visionStandardDeviation, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }

        buildPoseEstimator();
        configAutoBuilder();
    }

    public void buildPoseEstimator(){
        m_poseEstimator=new SwerveDrivePoseEstimator(getKinematics(),
                        new Rotation2d(this.getPigeon2().getYaw().getValue()),
                        this.getState().ModulePositions,getPose2d());
    }

    public void configAutoBuilder(){
        RobotConfig robotConfig=null;
        try{
            robotConfig = RobotConfig.fromGUISettings();
        }catch(Exception e){
            System.out.println("RobotConfig Error, Error: "+e);
        }
        try{
            AutoBuilder.configure(
                this::getPose2d,
                this::resetPose,
                this::getChassisSpeeds,
                this::setChassisSpeeds,
                Constants.pathPlanDriveController,
                robotConfig,
                ()->false,/*{
                    Alliance alliance=DriverStation.getAlliance().get();
                    if(alliance==Alliance.Red){
                        return true;
                    }else{
                        return false;
                    }
                },*/
                this
            );
        }catch(Exception e){
            System.out.println("AutoBuilder was not configured, Error: "+e);
        }
    }

    public void setChassisSpeeds(ChassisSpeeds chassisSpeeds,DriveFeedforwards driveFF){
        this.setControl(autoDrive.withSpeeds(chassisSpeeds)
        .withWheelForceFeedforwardsX(driveFF.robotRelativeForcesXNewtons())
        .withWheelForceFeedforwardsY(driveFF.robotRelativeForcesYNewtons()));
    }

    public ChassisSpeeds getChassisSpeeds(){
        return this.getKinematics().toChassisSpeeds(this.getState().ModuleStates);
    }

    public Pose2d getPose2d(){
        return getState().Pose;
    }

    public Command createPathCommand(PathPlannerPath path){
        return AutoBuilder.followPath(path);
    }

    public boolean checkCANConnections(){
        boolean returnValue=true;
        try{
            for(int i=0;i<this.getModules().length;i++){
                this.getModule(i).getDriveMotor();
                this.getModule(i).getSteerMotor();
                this.getModule(i).getEncoder();
            }
        }catch(Exception e){
            returnValue=false;
        }

        return returnValue;
    }

    public void drive(double forward, double strafe, double rot){
        SwerveRequest.RobotCentric newRequest=new SwerveRequest.RobotCentric();
        newRequest.withVelocityX(forward*TunerConstants.kAutoMaxSpeed).withVelocityY(strafe*TunerConstants.kAutoMaxSpeed)
        .withRotationalRate(rot*TunerConstants.kAutoMaxAngularSpeed);
        this.setControl(newRequest);
    }

    public void pointWheels(double direction){
        SwerveRequest.PointWheelsAt newRequest=new PointWheelsAt().withModuleDirection(new Rotation2d().fromDegrees(direction));

        this.setControl(newRequest);
    }

    public void brake(){
        SwerveRequest.SwerveDriveBrake newRequest=new SwerveDriveBrake();

        this.setControl(newRequest);
    }

    /**
     * Returns a command that applies the specified control request to this swerve drivetrain.
     *
     * @param request Function returning the request to apply
     * @return Command to run
     */
    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    /**
     * Returns a command that applies the specified control request to this swerve drivetrain.
     * 5454 Custom, Applies Gas Pedal
     * 
     * @param request Function returning the request to apply
     * @return Command to run
     */
    public Command applyRequestDrive(CommandXboxController driveController,int translationAxis,int strafeAxis,int rotationAxis) {
        SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric().withDeadband((TunerConstants.kMaxSpeed*0.1))
            .withRotationalDeadband((TunerConstants.kMaxAngularSpeed*0.1));

        //System.out.println("VELOCITY X SPEED: "+(-driveController.getRawAxis(translationAxis)*TunerConstants.kMaxSpeed)*m_gasPedalMult);
            
        return this.applyRequest(() -> drive.withVelocityX((-driveController.getRawAxis(translationAxis)*TunerConstants.kMaxSpeed)*m_gasPedalDriveMult)
            .withVelocityY((-driveController.getRawAxis(strafeAxis)*TunerConstants.kMaxSpeed)*m_gasPedalDriveMult)
            .withRotationalRate((-driveController.getRawAxis(rotationAxis)*TunerConstants.kMaxSpeed)*m_gasPedalRotMult)
        );
    }

    public void setGasPedalMult(double driveMult,double rotMult){
       // System.out.println("GAS PEDAL MULT: "+m_gasPedalMult);
        m_gasPedalDriveMult=driveMult;
        m_gasPedalRotMult=rotMult;
    }

    /**
     * Runs the SysId Quasistatic test in the given direction for the routine
     * specified by {@link #m_sysIdRoutineToApply}.
     *
     * @param direction Direction of the SysId Quasistatic test
     * @return Command to run
     */
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineToApply.quasistatic(direction);
    }

    /**
     * Runs the SysId Dynamic test in the given direction for the routine
     * specified by {@link #m_sysIdRoutineToApply}.
     *
     * @param direction Direction of the SysId Dynamic test
     * @return Command to run
     */
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineToApply.dynamic(direction);
    }

    @Override
    public void periodic() {
        
        
        /** Periodically try to apply the operator perspective.
         * If we haven't applied the operator perspective before, then we should apply it regardless of DS state.
         * This allows us to correct the perspective in case the robot code restarts mid-match.
         * Otherwise, only check and apply the operator perspective if the DS is disabled.
         * This ensures driving behavior doesn't change until an explicit disable event occurs during testing.
         */
        if (!m_hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
            DriverStation.getAlliance().ifPresent(allianceColor -> {
                setOperatorPerspectiveForward(
                    allianceColor == Alliance.Red
                        ? kRedAlliancePerspectiveRotation
                        : kBlueAlliancePerspectiveRotation
                );
                m_hasAppliedOperatorPerspective = true;
            });
        }

        m_poseEstimator.update(new Rotation2d(this.getPigeon2().getYaw().getValue()),
                                this.getState().ModulePositions);
   
        //Add telemtry
        Logger.recordOutput("SwerveDriveTrain/Yaw", this.getPigeon2().getYaw().getValue());
        Logger.recordOutput("SwerveDriveTrain/PoseEstimate",m_poseEstimator.getEstimatedPosition());
        Logger.recordOutput("SwerveDriveTrain/ModulePositions",this.getState().ModulePositions);                            }
                                                                                    
    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;

            /* use the measured time delta, get battery voltage from WPILib */
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }
}
