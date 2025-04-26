package frc.robot.subsystems;

import org.littletonrobotics.junction.LoggedRobot;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.util.DriveFeedforwards;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.RobotMap;
import frc.robot.classes.ObsidianCANSparkMax;
import frc.robot.common.drivers.Mk2SwerveModuleBuilder;
import frc.robot.common.drivers.NavX;
import frc.robot.common.drivers.SwerveModule;
import frc.robot.common.math.Vector2;

public class DrivetrainSubsystem extends SubsystemBase {

    private static final double TRACKWIDTH = 25;
    private static final double WHEELBASE = 31;
    private double m_gasPedalDriveMult=1;
    private double m_gasPedalRotMult=1;

  /*   private static final double FRONT_LEFT_ANGLE_OFFSET = -Math.toRadians(100);//(164.53+180); //30.6 last 6.5 - was 161.8
    private static final double FRONT_RIGHT_ANGLE_OFFSET = -Math.toRadians(100);
    private static final double BACK_LEFT_ANGLE_OFFSET = -Math.toRadians(60);//+180 22.1
    private static final double BACK_RIGHT_ANGLE_OFFSET = -Math.toRadians(100);//+180
*/
//USE RADIANS
//3.04 radians is half a rotation
// π/180
private static final double FRONT_LEFT_ANGLE_OFFSET = Math.toRadians(255);//0.084//4.04;//4.63-3.04; ///-0.850
public SwerveDrivePoseEstimator estimator; 


//private static final double FRONT_RIGHT_ANGLE_OFFSET = 0.9+3.04; //0.25; //-4.72+3.04;//-1.45
private static final double FRONT_RIGHT_ANGLE_OFFSET = Math.toRadians(282.5)+3.04;//0.5+3.04;1.02;// -0.73; //0.25; //-4.72+3.04;//-1.45
//private static final double FRONT_RIGHT_ANGLE_OFFSET=-Math.toRadians(205.4);
//private static final double BACK_LEFT_ANGLE_OFFSET = -1.25;//-77+3.04;//-0.78
//private static final double BACK_LEFT_ANGLE_OFFSET = 1.75;//-77+3.04;//-0.78
//private static final double BACK_LEFT_ANGLE_OFFSET = 1.95;//-77+3.04;//-0.78
//private static final double BACK_LEFT_ANGLE_OFFSET = -0.55-3.04;//-77+3.04;//-0.78
private static final double BACK_LEFT_ANGLE_OFFSET = Math.toRadians(223.2)+1.571;//-0.55-3.04;//-77+3.04;//-0.78

//private static final double BACK_RIGHT_ANGLE_OFFSET =-2.17-3.04; //-2.42-3.04
private static final double BACK_RIGHT_ANGLE_OFFSET = Math.toRadians(358.5);//-1.24-3.04; //-2.42-3.04
private SwerveRequest.ApplyRobotSpeeds autoDrive = new SwerveRequest.ApplyRobotSpeeds();


private boolean m_autoControl = false;

    private static DrivetrainSubsystem instance;

    private NavX m_gyroscope;
    private final SwerveModule frontLeftModule = new Mk2SwerveModuleBuilder(
            new Vector2(TRACKWIDTH / 2.0, WHEELBASE / 2.0))
            .angleEncoder(new AnalogInput(RobotMap.DRIVETRAIN_FRONT_LEFT_ANGLE_ENCODER), FRONT_LEFT_ANGLE_OFFSET)
            .angleMotor(new ObsidianCANSparkMax(RobotMap.DRIVETRAIN_FRONT_LEFT_ANGLE_MOTOR, MotorType.kBrushless,
            true),Mk2SwerveModuleBuilder.MotorType.NEO)
            .driveMotor(new ObsidianCANSparkMax(RobotMap.DRIVETRAIN_FRONT_LEFT_DRIVE_MOTOR, MotorType.kBrushless, true),
                    Mk2SwerveModuleBuilder.MotorType.NEO)
            .build();
    private final SwerveModule frontRightModule = new Mk2SwerveModuleBuilder(
            new Vector2(TRACKWIDTH / 2.0, -WHEELBASE / 2.0))
            .angleEncoder(new AnalogInput(RobotMap.DRIVETRAIN_FRONT_RIGHT_ANGLE_ENCODER), FRONT_RIGHT_ANGLE_OFFSET)
            .angleMotor(new ObsidianCANSparkMax(RobotMap.DRIVETRAIN_FRONT_RIGHT_ANGLE_MOTOR, MotorType.kBrushless,
                    true),Mk2SwerveModuleBuilder.MotorType.NEO)
            .driveMotor(new ObsidianCANSparkMax(RobotMap.DRIVETRAIN_FRONT_RIGHT_DRIVE_MOTOR, MotorType.kBrushless, true),
                    Mk2SwerveModuleBuilder.MotorType.NEO)
            .build();
    private final SwerveModule backLeftModule = new Mk2SwerveModuleBuilder(
            new Vector2(-TRACKWIDTH / 2.0, WHEELBASE / 2.0))
            .angleEncoder(new AnalogInput(RobotMap.DRIVETRAIN_BACK_LEFT_ANGLE_ENCODER), BACK_LEFT_ANGLE_OFFSET)
            .angleMotor(new ObsidianCANSparkMax(RobotMap.DRIVETRAIN_BACK_LEFT_ANGLE_MOTOR, MotorType.kBrushless, true),
                    Mk2SwerveModuleBuilder.MotorType.NEO)
            .driveMotor(new ObsidianCANSparkMax(RobotMap.DRIVETRAIN_BACK_LEFT_DRIVE_MOTOR, MotorType.kBrushless, true),
                    Mk2SwerveModuleBuilder.MotorType.NEO)
            .build();
    private final SwerveModule backRightModule = new Mk2SwerveModuleBuilder(
            new Vector2(-TRACKWIDTH / 2.0, -WHEELBASE / 2.0))
            .angleEncoder(new AnalogInput(RobotMap.DRIVETRAIN_BACK_RIGHT_ANGLE_ENCODER), BACK_RIGHT_ANGLE_OFFSET)
            .angleMotor(new ObsidianCANSparkMax(RobotMap.DRIVETRAIN_BACK_RIGHT_ANGLE_MOTOR, MotorType.kBrushless, true),
                    Mk2SwerveModuleBuilder.MotorType.NEO)
            .driveMotor(new ObsidianCANSparkMax(RobotMap.DRIVETRAIN_BACK_RIGHT_DRIVE_MOTOR, MotorType.kBrushless, true),
                    Mk2SwerveModuleBuilder.MotorType.NEO)
            .build();

    private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
            new Translation2d(TRACKWIDTH / 2.0, WHEELBASE / 2.0),
            new Translation2d(TRACKWIDTH / 2.0, -WHEELBASE / 2.0),
            new Translation2d(-TRACKWIDTH / 2.0, WHEELBASE / 2.0),
            new Translation2d(-TRACKWIDTH / 2.0, -WHEELBASE / 2.0)
    );

    /////////////////////////////////////////////////////////////////////////////


   

        
    public DrivetrainSubsystem(NavX navX) {
        m_gyroscope = navX;
        m_gyroscope.calibrate();
        m_gyroscope.setInverted(false); // You might not need to invert the gyro

        SwerveModulePosition frontLeftPosition=new SwerveModulePosition(frontLeftModule.getCurrentDistance(),new Rotation2d(frontLeftModule.getCurrentAngle()));
        SwerveModulePosition frontRightPosition=new SwerveModulePosition(frontRightModule.getCurrentDistance(),new Rotation2d(frontRightModule.getCurrentAngle()));
        SwerveModulePosition backleftPosition=new SwerveModulePosition(backLeftModule.getCurrentDistance(),new Rotation2d(backLeftModule.getCurrentAngle()));
        SwerveModulePosition backRightPosition=new SwerveModulePosition(backRightModule.getCurrentDistance(),new Rotation2d(backRightModule.getCurrentAngle()));
        estimator = new SwerveDrivePoseEstimator(kinematics,getGyroscopeRotation(),
                new SwerveModulePosition[] { 
                        frontLeftPosition,
                        frontRightPosition,
                        backleftPosition,
                        backRightPosition
                      },new Pose2d()); 

                //Logger.getInstance().recordOutput("Odometry X", estimator.getEstimatedPosition().getX());
                //Logger.getInstance().recordOutput("Odometry Y", estimator.getEstimatedPosition().getY());
 
       
        frontLeftModule.setName("Front Left");
        frontRightModule.setName("Front Right");
        backLeftModule.setName("Back Left");
        backRightModule.setName("Back Right");
    }




    public Pose2d getPose2d(){
        return estimator.getEstimatedPosition();
    }

    public void resetpose(Pose2d pose){
         estimator.resetPose(getPose2d());
    }
    
    public ChassisSpeeds getChassisSpeeds(){
        return this.kinematics.toChassisSpeeds();
    }

    public void setChassisSpeeds(ChassisSpeeds chassisSpeeds, DriveFeedforwards driveFF){
        this.autoDrive.withSpeeds(chassisSpeeds)
        .withWheelForceFeedforwardsX(driveFF.robotRelativeForcesXNewtons())
        .withWheelForceFeedforwardsY(driveFF.robotRelativeForcesYNewtons());
    }

    public void configAutoBuilder(){
        RobotConfig robotConfig = null;
        try{
                robotConfig = RobotConfig.fromGUISettings();
        } catch (Exception e) {
        // Handle exception as needed
                System.out.println("RobotConfig Error, Error: "+e);
        } try{
                AutoBuilder.configure(
                        this::getPose2d, //pose and chassis speed should work now
                        this::resetpose,
                        this::getChassisSpeeds,
                        this::setChassisSpeeds,
                        Constants.pathPlanDriveController,
                        robotConfig,
                        ()->{
                            Alliance alliance=DriverStation.getAlliance().get();
                            if(alliance == Alliance.Red){
                                return true;
                            }else{
                                return false;
                            }
                        },
                        this
                    );
        } catch(Exception e){
                System.out.println("AutoBuilder was not configured, Error: "+e);
        }

    }
 
    public static DrivetrainSubsystem getInstance() {
        if (instance == null) {
            instance = new DrivetrainSubsystem(new NavX(SPI.Port.kMXP));
        }

        return instance;
    }
 
    public boolean IsRobotMoving(){
            if((backLeftModule.getCurrentVelocity()+
                backRightModule.getCurrentVelocity()+
                frontLeftModule.getCurrentVelocity()+
                frontRightModule.getCurrentVelocity())==0) {
                        return false;
                }else {
                        return true;
                }
    }
    public void movenodistance(double direction, double rotation, double speed){
        move(direction,rotation,speed,0,false);
    }
    public void move(double direction, double rotation,double speed, double distance, boolean stopAtEnd){
        automove(direction,rotation,speed,distance,stopAtEnd,false);
    }
    public void moveGyro(double direction, double rotation,double speed, double distance, boolean stopAtEnd){
        automove(direction,rotation,speed,distance,stopAtEnd,true);
    }
    public void moveGyroNodistance(double direction, double rotation, double speed){
        automove(direction,rotation,speed,0,false,true);
    }

    public double legacyGetDistancefromWheel(){
        return 0;//TODO:GetDistance
    }
    public void stopAutoDrive(){
        drive(new Translation2d(0,0),0,isFieldCentric());
        m_autoControl=false;
    }
    
    private void automove(double direction, double rotation,double speed, double distance, boolean stopAtEnd,boolean fieldCentric){       
        double startDistance;
        double forward=0;
        double strafe=0;
        Translation2d targetTranslation;
        m_autoControl = true;
        startDistance=backLeftModule.getCurrentDistance();
        switch ((int) direction){
                case 0:
                        forward=-1*speed;
                        strafe=0;
                        break;
                case 45:
                        forward=-1*speed;
                        strafe=1*speed;
                        break;
               
                case 90:
                        forward=0;
                        strafe=1*speed;
                        break;
                case 135:
                        forward=1*speed;
                        strafe=1*speed;
                        break;
                case 180:
                        forward=1*speed;
                        strafe=0;
                        break;
                case 225:
                        forward=1*speed;
                        strafe=-1*speed;
                        break;
                case 270:
                        forward=0;
                        strafe=-1*speed;
                        break;
                case 315:
                        forward=-1*speed;
                        strafe=-1*speed;
                        break;
                
        }
        //if not travelling distance then just turn movement on 
        if(distance==0){
                drive(new Translation2d(forward, strafe), rotation, fieldCentric);
                periodic();
        }else {
                double distanceTravelled=backLeftModule.getCurrentDistance()-startDistance;
                while(distanceTravelled<=distance && m_autoControl){
                drive(new Translation2d(forward, strafe), rotation, fieldCentric);
                periodic();
                distanceTravelled=Math.abs(backLeftModule.getCurrentDistance()-startDistance);
                 System.out.print("(" + forward + ", "+ strafe +") " + distanceTravelled + " / " + distance );
                } 
        }
        if (stopAtEnd) {
                drive(new Translation2d(0,0), 0, fieldCentric);
                periodic();                
        }
}

public void setGasPedalMult(double driveMult, double rotMult){
        m_gasPedalDriveMult = driveMult;
        m_gasPedalRotMult = rotMult;
}

public void stop(){
        drive(new Translation2d(0,0), 0, true);
        periodic();       
}
public double getGyroAngle()
{
        return m_gyroscope.getAngle().toDegrees();
}
public void spinLeft(double speed){
        drive(new Translation2d(0, 0), -speed,true);

}
public void spinRight(double speed){
        drive(new Translation2d(0, 0), speed,true);
              
}

public void spin (double direction,double speed)
{       double startAngle;
        double endAngle;
        double forward=0;
        double strafe=0;
        Translation2d targetTranslation;
        startAngle=m_gyroscope.getAngle().toDegrees();
        endAngle=direction+startAngle;
        
        do {
              drive(new Translation2d(0, 0), -30,false);
              periodic();
              // System.out.print("(" + forward + ", "+ strafe +") " + distanceTravelled + " / " + distance );
        } while(m_gyroscope.getAngle().toDegrees()<=endAngle);
       //stop driving
        drive(new Translation2d(0,0), 0, true);
        periodic();                

}
    @Override
    public void periodic() {
        frontLeftModule.updateSensors();
        frontRightModule.updateSensors();
        backLeftModule.updateSensors();
        backRightModule.updateSensors();
    
        SmartDashboard.putNumber("Front Left Module Angle", Math.toDegrees(frontLeftModule.getCurrentAngle()));
        SmartDashboard.putNumber("Front Right Module Angle", Math.toDegrees(frontRightModule.getCurrentAngle()));
        SmartDashboard.putNumber("Back Left Module Angle", Math.toDegrees(backLeftModule.getCurrentAngle()));
        SmartDashboard.putNumber("Back Right Module Angle", Math.toDegrees(backRightModule.getCurrentAngle()));
        SmartDashboard.putNumber("Front Left Module Radian", frontLeftModule.getCurrentAngle());
        SmartDashboard.putNumber("Front Right Module Radian", frontRightModule.getCurrentAngle());
        SmartDashboard.putNumber("Back Left Module Radain", backLeftModule.getCurrentAngle());
        SmartDashboard.putNumber("Back Right Module Radain", backRightModule.getCurrentAngle());

        SmartDashboard.putNumber("Gyroscope Angle", m_gyroscope.getAngle().toDegrees());

        
        frontLeftModule.updateState(LoggedRobot.defaultPeriodSecs);
        frontRightModule.updateState(LoggedRobot.defaultPeriodSecs);
        backLeftModule.updateState(LoggedRobot.defaultPeriodSecs);
        backRightModule.updateState(LoggedRobot.defaultPeriodSecs);

        SwerveModulePosition frontLeftPosition=new SwerveModulePosition(frontLeftModule.getCurrentDistance(),new Rotation2d(frontLeftModule.getCurrentAngle()));
        SwerveModulePosition frontRightPosition=new SwerveModulePosition(frontRightModule.getCurrentDistance(),new Rotation2d(frontRightModule.getCurrentAngle()));
        SwerveModulePosition backleftPosition=new SwerveModulePosition(backLeftModule.getCurrentDistance(),new Rotation2d(backLeftModule.getCurrentAngle()));
        SwerveModulePosition backRightPosition=new SwerveModulePosition(backRightModule.getCurrentDistance(),new Rotation2d(backRightModule.getCurrentAngle()));
      //  estimator.update(getGyroscopeRotation(),  new SwerveModulePosition[] { 
       //         frontLeftPosition,
       //         frontRightPosition,
       ////         backleftPosition,
        //        backRightPosition
         //     });
        
              //System.out.println("Current Pose: " + estimator.getEstimatedPosition().toString());
    }

    public void drive(Translation2d translation, double rotation, boolean fieldOriented) {
        //Logger.getInstance().recordOutput("Drive s",translation.toString());
        rotation *= 2.0 / Math.hypot(WHEELBASE, TRACKWIDTH);
        ChassisSpeeds speeds;
        if (fieldOriented) {
            speeds = ChassisSpeeds.fromFieldRelativeSpeeds(translation.getX(), translation.getY(), rotation,
                    Rotation2d.fromDegrees(m_gyroscope.getAngle().toDegrees()));
                System.out.println(speeds);
        } else {
            speeds = new ChassisSpeeds(translation.getX(), translation.getY(), rotation);
        }
 
        SwerveModuleState[] states = kinematics.toSwerveModuleStates(speeds);
        frontLeftModule.setTargetVelocity(states[0].speedMetersPerSecond, states[0].angle.getRadians());
        frontRightModule.setTargetVelocity(states[1].speedMetersPerSecond, states[1].angle.getRadians());
        backLeftModule.setTargetVelocity(states[2].speedMetersPerSecond, states[2].angle.getRadians());
        backRightModule.setTargetVelocity(states[3].speedMetersPerSecond, states[3].angle.getRadians());
        //System.out.println("Gyro " + m_gyroscope.getAngle().toDegrees() + " " + frontLeftModule.getCurrentAngle() + " " + fieldOriented);

}
public boolean isNotInAutoControl(){
        return m_autoControl==false;
}
//TOODO:
    public double getFrontLeftAngleDegrees(){
        return frontLeftModule.getCurrentAngle();
    }
    public double getFrontRightAngleDegrees(){
        return frontRightModule.getCurrentAngle();
    }
    public double getBackLeftAngleDegrees(){
        return backLeftModule.getCurrentAngle();
    }
    public double getbackRightAngleDegrees(){
        return backRightModule.getCurrentAngle();
    }
   

    public double getFrontLeftAngle(){
        return frontLeftModule.getCurrentAngle();
    }
    public double getFrontRightAngle(){
        return frontRightModule.getCurrentAngle();
    }
    public double getBackLeftAngle(){
        return backLeftModule.getCurrentAngle();
    }
    public double getbackRightAngle(){
        return backRightModule.getCurrentAngle();
    }

    private Rotation2d getGyroscopeRotation() {
        return Rotation2d.fromDegrees(m_gyroscope.getYaw());
    }
    public void resetGyroscope() { 
        m_gyroscope.setAdjustmentAngle(m_gyroscope.getUnadjustedAngle());
    }

    public boolean isFieldCentric(){
        return true;
    }
    
    public void resetDriveMode(){
        m_autoControl = false;
        stop();
    }
}