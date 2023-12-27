package frc.robot.subsystems;

import frc.robot.common.drivers.WPISwerveModule;
import edu.wpi.first.wpilibj.AnalogInput;
import java.util.function.ToDoubleFunction;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.common.drivers.NavX;

public class WPIDriveTrainSubsystem extends SubsystemBase{
    private static final double TRACKWIDTH = 25;
    private static final double WHEELBASE = 31;
    public double speedMultiplier=1;

    private NavX m_gyro;

    public SwerveDriveKinematics m_driveKinematics = new SwerveDriveKinematics(
            new Translation2d(TRACKWIDTH / 2.0, WHEELBASE / 2.0),
            new Translation2d(TRACKWIDTH / 2.0, -WHEELBASE / 2.0),
            new Translation2d(-TRACKWIDTH / 2.0, WHEELBASE / 2.0),
            new Translation2d(-TRACKWIDTH / 2.0, -WHEELBASE / 2.0)
    );

    private final WPISwerveModule m_frontLeftModule=new WPISwerveModule(this,25, 
                                    20, 2);
    private final WPISwerveModule m_frontRightModule=new WPISwerveModule(this,26, 
                                    21, 1);
    private final WPISwerveModule m_backLeftModule=new WPISwerveModule(this,23, 
                                    28, 3);
    private final WPISwerveModule m_backRightModule=new WPISwerveModule(this,22, 
                                    27, 0);

    public WPIDriveTrainSubsystem(NavX gyro){
        m_gyro=gyro;
    }

    public SwerveModulePosition[] getModulePositions(){
        return new SwerveModulePosition[]{m_frontLeftModule.getPosition(),m_frontRightModule.getPosition(),
            m_backLeftModule.getPosition(),m_backRightModule.getPosition()
        };
    }

    public Rotation2d getRotation2D(){
        return Rotation2d.fromDegrees(m_gyro.getAngleAsDouble());
    }

    public void resetAllDriveEncoders(){
        m_frontLeftModule.resetDriveEncoder();
        m_frontRightModule.resetDriveEncoder();
        m_backLeftModule.resetDriveEncoder();
        m_backRightModule.resetDriveEncoder();
    }

    public void stopAllModules(){
        m_frontLeftModule.stop();
        m_frontRightModule.stop();
        m_backLeftModule.stop();
        m_backRightModule.stop();
    }

    public double getRobotDegrees(){
        double rawValue=m_gyro.getAngleAsDouble()%360;

        if(rawValue>0.0){
            return rawValue+360;
        }else{
            return rawValue;
        }
    }

    public double newHeading(){
        return m_gyro.getAngleAsDouble();
    }

    public void setModuleStates(SwerveModuleState[] desiredStates){
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates,Constants.WPISwerve.physicalMaxSpeedMetersPerSecond);
        //System.out.println("DesaturationY"+desiredStates[0].angle.getDegrees());

        m_frontLeftModule.setState(desiredStates[0]);
        m_frontRightModule.setState(desiredStates[1]);
        m_backLeftModule.setState(desiredStates[2]);
        m_backRightModule.setState(desiredStates[3]);
    }
}
