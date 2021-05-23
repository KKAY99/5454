/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.classes.drives.swervewpi;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveOdometry;
import frc.robot.classes.constants.Constants.SwerveModules;
import frc.robot.classes.utilfuncs.UtilFuncs;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SlewRateLimiter;

/**
 * Represents a swerve drive style drivetrain.
 */
public class WpiSwerveDrive {

//    public static final int frontLeftOffset = -172;
//    public static final int frontRightOffset = -434;
    
//    public static final int backLeftOffset = -127;
//    public static final int backRightOffset = -936;
    public static final int frontLeftOffset = -503;
    public static final int frontRightOffset = -1280;
    
    public static final int backLeftOffset = 374;
    public static final int backRightOffset = 94;

    private double m_FWD = 0.0;
    private double m_STR = 0.0;
    private double m_RCW = 0.0;

    private static final double inchesToMeters = 0.0254;

    public static final double kMaxDriveSpeed = 10.0; // feet per second
    public static final double kConvertedMaxDriveSpeed = kMaxDriveSpeed / 3.281; // this converts to meters per second

    public static final double kMaxSpinSpeed = 0.5; // rotations per second
    public static final double kConvertedMaxSpinSpeed = kMaxSpinSpeed * (2.0 / Math.PI); // convert to radians per
                                                                                         // second

    private static final double trackWidth = 27.0; // Left to Right wheel C-C in inches
    private static final double wheelBase = 24.5; // Front to Back wheel C-C in inches

    private static final double convertedTrackWidth = trackWidth * inchesToMeters;
    private static final double convertedWheelBase = wheelBase * inchesToMeters;

    private static final double halfConvertedTrackWidth = convertedTrackWidth * 0.5;
    private static final double halfConvertedWheelBase = convertedWheelBase * 0.5;

    private final Translation2d m_frontLeftLocation = new Translation2d(halfConvertedTrackWidth,
            halfConvertedWheelBase);
    private final Translation2d m_frontRightLocation = new Translation2d(halfConvertedTrackWidth,
            -halfConvertedWheelBase);
    private final Translation2d m_backLeftLocation = new Translation2d(-halfConvertedTrackWidth,
            halfConvertedWheelBase);
    private final Translation2d m_backRightLocation = new Translation2d(-halfConvertedTrackWidth,
            -halfConvertedWheelBase);

    private SwerveModule m_frontLeft = new SwerveModule(7, 1, frontLeftOffset);
    private SwerveModule m_frontRight = new SwerveModule(4, 3, frontRightOffset);
  
    private SwerveModule m_backLeft = new SwerveModule(5, 2, backLeftOffset);
    private SwerveModule m_backRight = new SwerveModule(6, 0, backRightOffset);  
    private AHRS m_gyro = new AHRS(SPI.Port.kMXP);
   // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
   final SlewRateLimiter m_xspeedLimiter = new SlewRateLimiter(3);
   final SlewRateLimiter m_yspeedLimiter = new SlewRateLimiter(3);
   final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(3);

    private final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(m_frontLeftLocation,
            m_frontRightLocation, m_backLeftLocation, m_backRightLocation);

    private final SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(m_kinematics, getAngle());

    public WpiSwerveDrive() {
        m_gyro.reset();
    }

    /**
     * Returns the angle of the robot as a Rotation2d.
     *
     * @return The angle of the robot.
     */
    public Rotation2d getAngle() {
        // Negating the angle because WPILib gyros are CW positive.
        return Rotation2d.fromDegrees(-m_gyro.getAngle());
    }

    /**
     * Method to drive the robot using joystick info.
     *
     * @param xSpeed        Speed of the robot in the x direction (forward).
     * @param ySpeed        Speed of the robot in the y direction (sideways).
     * @param rot           Angular rate of the robot.
     * @param fieldRelative Whether the provided x and y speeds are relative to the
     *                      field.
     */
    @SuppressWarnings("ParameterName")
    public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
        var swerveModuleStates = m_kinematics.toSwerveModuleStates(
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, getAngle())
                        : new ChassisSpeeds(xSpeed, ySpeed, rot));
        System.out.println("angle " + swerveModuleStates[0].angle.toString());
        SwerveDriveKinematics.normalizeWheelSpeeds(swerveModuleStates, kConvertedMaxDriveSpeed);
        
        System.out.println("Front left " + m_frontLeft.getOffset());
        m_frontLeft.setDesiredState(swerveModuleStates[0]);
        m_frontRight.setDesiredState(swerveModuleStates[1]);
        m_backLeft.setDesiredState(swerveModuleStates[2]);           
        m_backRight.setDesiredState(swerveModuleStates[3]);
    }

    /**
     * Method to drive the robot using joystick info.
     *
     * @param RCW                   Angular rate of the robot.
     * @param FWD                   Speed of the robot in the x direction (forward).
     * @param STR                   Speed of the robot in the y direction
     *                              (sideways).
     * @param fieldRelative         Whether the provided x and y speeds are relative
     *                              to the field.
     * @param shouldEnableDeadzones Whether the provided joysticks values should use
     *                              a deadzone.
     */
    public void driveWithJoystick(double RCW, double FWD, double STR, boolean fieldCentric,
            boolean shouldEnableDeadzones) {

        FWD = UtilFuncs.deadzoneModify(FWD, .1);
        STR = UtilFuncs.deadzoneModify(STR, .1);
        RCW = UtilFuncs.deadzoneModify(RCW, .1);

        m_FWD = FWD;
        m_STR = STR;
        m_RCW = RCW;

        
     
        final var xSpeed = m_xspeedLimiter.calculate(FWD * kConvertedMaxDriveSpeed);

         final var ySpeed = m_yspeedLimiter.calculate(STR * kConvertedMaxDriveSpeed);
         
        final var rot = m_rotLimiter.calculate(RCW * kConvertedMaxSpinSpeed);
       
        
        drive(xSpeed, ySpeed, rot, fieldCentric);
    }

    /**
     * Kills all motors of the drive
     */
    public void driveKill() {
        System.out.println("Drive Kill");
        driveWithJoystick(0, 0, 0, true, false);
    }

    /**
     * Updates the field relative position of the robot.
     */
    public void updateOdometry() {
        m_odometry.update(getAngle(), m_frontLeft.getState(), m_frontRight.getState(), m_backLeft.getState(),
                m_backRight.getState());
    }

    /**
     * Returns the forward value for the robot
     */
    public double getDriveFWD() {
        return m_FWD;
    }

    /**
     * Returns the stafe value for the robot
     */
    public double getDriveSTR() {
        return m_STR;
    }
    public double getSteerEncoderActual (int swerveModule) {
        double returnValue=0;
        switch (swerveModule){
            case SwerveModules.kFrontLeft: {
                returnValue= m_frontLeft.getSteerEncoderActual();   
                break;
            }
            case SwerveModules.kFrontRight: {
                returnValue= m_frontRight.getSteerEncoderActual();   
                break; 
            }
            case SwerveModules.kBackLeft: {
                returnValue= m_backLeft.getSteerEncoderActual();    
                break;
            }
            case SwerveModules.kBackRight: {
                returnValue= m_backRight.getSteerEncoderActual(); 
                break;
            }                      
        }
        return returnValue;
    }   
    public double getSteerEncoderTarget (int swerveModule) {
        double returnValue=0;
        switch (swerveModule){
            
            case SwerveModules.kFrontLeft: {
                returnValue= m_frontLeft.getSteerEncoderTarget();    
                break;
            }
            case SwerveModules.kFrontRight: {
                returnValue=m_frontRight.getSteerEncoderTarget(); 
                break;
           }
            case SwerveModules.kBackLeft: {
                returnValue= m_backLeft.getSteerEncoderTarget();    
                break;}
            case SwerveModules.kBackRight: {
                returnValue= m_backRight.getSteerEncoderTarget();    
                break;
            }
        }
        return returnValue;
    }
    public double getDegrees(int swerveModule){
        double returnValue=0;
          switch (swerveModule){
              case SwerveModules.kFrontLeft: {
                  returnValue= m_frontLeft.getState().angle.getDegrees();   
                  break;
              }
              case SwerveModules.kFrontRight: {
                  returnValue= m_frontRight.getState().angle.getDegrees();   
                  break; 
              }
              case SwerveModules.kBackLeft: {
                  returnValue= m_backLeft.getState().angle.getDegrees();
                  break;
              }
              case SwerveModules.kBackRight: {
                  returnValue= m_backRight.getState().angle.getDegrees(); 
                  break;
              }                      
          }
      return returnValue;
      } 
      public double getSpeed(int swerveModule){
        double returnValue=0;
          switch (swerveModule){
              case SwerveModules.kFrontLeft: {
                  returnValue= m_frontLeft.getState().speedMetersPerSecond;   
                  break;
              }
              case SwerveModules.kFrontRight: {
                  returnValue= m_frontRight.getState().speedMetersPerSecond;   
                  break; 
              }
              case SwerveModules.kBackLeft: {
                  returnValue= m_backLeft.getState().speedMetersPerSecond;
                  break;
              }
              case SwerveModules.kBackRight: {
                  returnValue= m_backRight.getState().speedMetersPerSecond; 
                  break;
              }                      
          }
      return returnValue;
      } 
    /**
     * Returns the rotate value for the robot
     */
  
     public void moveWheel(int swerveModule,double speed){
        switch (swerveModule){
            case SwerveModules.kFrontLeft: {
                 m_frontLeft.spinWheel(speed);
                 break;
            }
            case SwerveModules.kFrontRight: {
                m_frontRight.spinWheel(speed);
                break;
            }
            case SwerveModules.kBackLeft: {
                m_backLeft.spinWheel(speed);
                break;
            }
            case SwerveModules.kBackRight: {
                m_backRight.spinWheel(speed);
                break;
            }
    }                          

     }
     public double getDriveRCW() {
        return m_RCW;
    }
}