package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import frc.robot.Constants.RobotMap;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.hal.ThreadsJNI;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;     
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
 import frc.robot.common.drivers.SwerveModule;
import frc.robot.common.math.Vector2;
import frc.robot.common.drivers.Mk2SwerveModuleBuilder;
import frc.robot.common.drivers.NavX;
 
public class DrivetrainSubsystem extends SubsystemBase {
    private static final double TRACKWIDTH = 20;
    private static final double WHEELBASE = 25;

    private static final double FRONT_LEFT_ANGLE_OFFSET = -Math.toRadians(349-180);//(164.53+180); //30.6 last 6.5 - was 161.8
    private static final double FRONT_RIGHT_ANGLE_OFFSET = -Math.toRadians(299.54+180);
    private static final double BACK_LEFT_ANGLE_OFFSET = -Math.toRadians(22.1);//+180
    private static final double BACK_RIGHT_ANGLE_OFFSET = -Math.toRadians(69.84);//+180

    private static DrivetrainSubsystem instance;

    private NavX m_gyroscope;
    private final SwerveModule frontLeftModule = new Mk2SwerveModuleBuilder(
            new Vector2(TRACKWIDTH / 2.0, WHEELBASE / 2.0))
            .angleEncoder(new AnalogInput(RobotMap.DRIVETRAIN_FRONT_LEFT_ANGLE_ENCODER), FRONT_LEFT_ANGLE_OFFSET)
            .angleMotor(new CANSparkMax(RobotMap.DRIVETRAIN_FRONT_LEFT_ANGLE_MOTOR, CANSparkMaxLowLevel.MotorType.kBrushless),
                    Mk2SwerveModuleBuilder.MotorType.NEO)
            .driveMotor(new CANSparkMax(RobotMap.DRIVETRAIN_FRONT_LEFT_DRIVE_MOTOR, CANSparkMaxLowLevel.MotorType.kBrushless),
                    Mk2SwerveModuleBuilder.MotorType.NEO)
            .build();
    private final SwerveModule frontRightModule = new Mk2SwerveModuleBuilder(
            new Vector2(TRACKWIDTH / 2.0, -WHEELBASE / 2.0))
            .angleEncoder(new AnalogInput(RobotMap.DRIVETRAIN_FRONT_RIGHT_ANGLE_ENCODER), FRONT_RIGHT_ANGLE_OFFSET)
            .angleMotor(new CANSparkMax(RobotMap.DRIVETRAIN_FRONT_RIGHT_ANGLE_MOTOR, CANSparkMaxLowLevel.MotorType.kBrushless),
                    Mk2SwerveModuleBuilder.MotorType.NEO)
            .driveMotor(new CANSparkMax(RobotMap.DRIVETRAIN_FRONT_RIGHT_DRIVE_MOTOR, CANSparkMaxLowLevel.MotorType.kBrushless),
                    Mk2SwerveModuleBuilder.MotorType.NEO)
            .build();
    private final SwerveModule backLeftModule = new Mk2SwerveModuleBuilder(
            new Vector2(-TRACKWIDTH / 2.0, WHEELBASE / 2.0))
            .angleEncoder(new AnalogInput(RobotMap.DRIVETRAIN_BACK_LEFT_ANGLE_ENCODER), BACK_LEFT_ANGLE_OFFSET)
            .angleMotor(new CANSparkMax(RobotMap.DRIVETRAIN_BACK_LEFT_ANGLE_MOTOR, CANSparkMaxLowLevel.MotorType.kBrushless),
                    Mk2SwerveModuleBuilder.MotorType.NEO)
            .driveMotor(new CANSparkMax(RobotMap.DRIVETRAIN_BACK_LEFT_DRIVE_MOTOR, CANSparkMaxLowLevel.MotorType.kBrushless),
                    Mk2SwerveModuleBuilder.MotorType.NEO)
            .build();
    private final SwerveModule backRightModule = new Mk2SwerveModuleBuilder(
            new Vector2(-TRACKWIDTH / 2.0, -WHEELBASE / 2.0))
            .angleEncoder(new AnalogInput(RobotMap.DRIVETRAIN_BACK_RIGHT_ANGLE_ENCODER), BACK_RIGHT_ANGLE_OFFSET)
            .angleMotor(new CANSparkMax(RobotMap.DRIVETRAIN_BACK_RIGHT_ANGLE_MOTOR, CANSparkMaxLowLevel.MotorType.kBrushless),
                    Mk2SwerveModuleBuilder.MotorType.NEO)
            .driveMotor(new CANSparkMax(RobotMap.DRIVETRAIN_BACK_RIGHT_DRIVE_MOTOR, CANSparkMaxLowLevel.MotorType.kBrushless),
                    Mk2SwerveModuleBuilder.MotorType.NEO)
            .build();

    private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
            new Translation2d(TRACKWIDTH / 2.0, WHEELBASE / 2.0),
            new Translation2d(TRACKWIDTH / 2.0, -WHEELBASE / 2.0),
            new Translation2d(-TRACKWIDTH / 2.0, WHEELBASE / 2.0),
            new Translation2d(-TRACKWIDTH / 2.0, -WHEELBASE / 2.0)
    );

        
    public DrivetrainSubsystem(NavX navX) {
        m_gyroscope = navX;
        m_gyroscope.calibrate();
        m_gyroscope.setInverted(true); // You might not need to invert the gyro

        frontLeftModule.setName("Front Left");
        frontRightModule.setName("Front Right");
        backLeftModule.setName("Back Left");
        backRightModule.setName("Back Right");
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
    public void stop(){
        drive(new Translation2d(0,0), 0, true);
        periodic();       
}
    public void move (double direction, double rotation,double speed, double distance, boolean stopAtEnd)
{       double startDistance;
        double forward=0;
        double strafe=0;
        Translation2d targetTranslation;
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
              drive(new Translation2d(forward, strafe), rotation, false);
              periodic();
              distanceTravelled=Math.abs(backLeftModule.getCurrentDistance()-startDistance);
              System.out.print("(" + forward + ", "+ strafe +") " + distanceTravelled + " / " + distance );
        } while(distanceTravelled<=distance);
        if (stopAtEnd) {
                drive(new Translation2d(0,0), 0, true);
                periodic();                
        }
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
              drive(new Translation2d(0, 0), 30,false);
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

        SmartDashboard.putNumber("Gyroscope Angle", m_gyroscope.getAngle().toDegrees());

        frontLeftModule.updateState(TimedRobot.kDefaultPeriod);
        frontRightModule.updateState(TimedRobot.kDefaultPeriod);
        backLeftModule.updateState(TimedRobot.kDefaultPeriod);
        backRightModule.updateState(TimedRobot.kDefaultPeriod);
    }

    public void drive(Translation2d translation, double rotation, boolean fieldOriented) {
        rotation *= 2.0 / Math.hypot(WHEELBASE, TRACKWIDTH);
        ChassisSpeeds speeds;
        if (fieldOriented) {
            speeds = ChassisSpeeds.fromFieldRelativeSpeeds(translation.getX(), translation.getY(), rotation,
                    Rotation2d.fromDegrees(m_gyroscope.getAngle().toDegrees()));
        } else {
            speeds = new ChassisSpeeds(translation.getX(), translation.getY(), rotation);
        }

        SwerveModuleState[] states = kinematics.toSwerveModuleStates(speeds);

        frontLeftModule.setTargetVelocity(states[0].speedMetersPerSecond, states[0].angle.getRadians());
        frontRightModule.setTargetVelocity(states[1].speedMetersPerSecond, states[1].angle.getRadians());
        backLeftModule.setTargetVelocity(states[2].speedMetersPerSecond, states[2].angle.getRadians());
        backRightModule.setTargetVelocity(states[3].speedMetersPerSecond, states[3].angle.getRadians());
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
    public void resetGyroscope() {
        
        m_gyroscope.setAdjustmentAngle(m_gyroscope.getUnadjustedAngle());
    }
}