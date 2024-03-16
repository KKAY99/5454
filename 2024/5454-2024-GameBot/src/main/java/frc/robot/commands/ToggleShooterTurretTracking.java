package frc.robot.commands;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants;
import frc.robot.Constants.LimeLightValues;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.SmartShooterConstants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.Swerve;
import frc.robot.utilities.FieldRelativeAccel;
import frc.robot.utilities.FieldRelativeSpeed;
import frc.robot.utilities.Limelight;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.utilities.ShotTable;
import edu.wpi.first.wpilibj.DriverStation;

public class ToggleShooterTurretTracking extends Command {
    private final ShooterSubsystem m_shooter;
    private final TurretSubsystem m_turret;
    private final Swerve m_drive;
    private final IntakeSubsystem m_intake;
    private final Limelight m_limelight;
    private final Timer m_timer = new Timer();
    private ShotTable m_shotTable = new ShotTable();

    private double m_lastDis;

    private enum SHOOTERSTATES{
        VISIONCLEARANCE,WAITFORVISIONCLEARANCE,SETANGLE,WAITFORANGLE
    }

    private SHOOTERSTATES m_shooterState=SHOOTERSTATES.VISIONCLEARANCE;

    public ToggleShooterTurretTracking(ShooterSubsystem shooter, TurretSubsystem turret, Swerve drive, Limelight limelight,IntakeSubsystem intake) {
        m_shooter = shooter;
        m_turret = turret;
        m_drive = drive;
        m_limelight = limelight;
        m_intake=intake;
        addRequirements(shooter,turret,intake);
    }


    @Override
    public void initialize() {
        m_shooter.SlowShootingMotors();
        m_timer.reset();
        m_timer.start();
        m_shooterState=SHOOTERSTATES.VISIONCLEARANCE;
    }
   

    @Override
    public boolean isFinished(){
        double limeLimelightDis=m_limelight.getDistance();
        double angleGap=0;

        m_turret.TrackTarget(true);

        switch(m_shooterState){
            case VISIONCLEARANCE: 
            if(!m_limelight.isTargetAvailible()){

                if(m_shooter.getRelativePosition()<ShooterConstants.shooterVisionClearanceAngle){
                    m_shooter.setAngle(ShooterConstants.shooterVisionClearanceAngle);
                    m_shooterState=SHOOTERSTATES.WAITFORVISIONCLEARANCE;
                }else{
                    m_shooterState=SHOOTERSTATES.SETANGLE;
                }
            }
            break;
            case WAITFORVISIONCLEARANCE: 
            angleGap=Math.abs(m_shooter.getRelativePosition())
                    -Math.abs(ShooterConstants.shooterVisionClearanceAngle);
                Logger.recordOutput("Shooter/AngleGap",angleGap);
                if(angleGap<Constants.ShooterConstants.kAngleDeadband && angleGap>-Constants.ShooterConstants.kAngleDeadband){
                    m_shooter.stopRotate();
                    Logger.recordOutput("Shooter/AngleGap",0);
                }
            break;
            case SETANGLE: 
                if(m_limelight.isTargetAvailible()){  
                    m_lastDis=limeLimelightDis;  
                    m_shooter.setAngle(m_shotTable.getAngle(limeLimelightDis));
                    m_shooterState=SHOOTERSTATES.WAITFORANGLE;
                }else{
                    m_shooter.stopRotate();
                    m_shooterState=SHOOTERSTATES.SETANGLE;
                }
            break;
            case WAITFORANGLE:
                angleGap=Math.abs(m_shooter.getRelativePosition())
                    -Math.abs(m_shotTable.getAngle(limeLimelightDis));
                Logger.recordOutput("Shooter/AngleGap",angleGap);
                if(angleGap<Constants.ShooterConstants.kAngleDeadband&&angleGap>-Constants.ShooterConstants.kAngleDeadband){
                    m_shooter.stopRotate();
                    Logger.recordOutput("Shooter/AngleGap",0);
                }

                if(m_lastDis>limeLimelightDis+LimeLightValues.limelightLastDisDeadband||m_lastDis<limeLimelightDis-LimeLightValues.limelightLastDisDeadband){
                    m_shooterState=SHOOTERSTATES.SETANGLE;
                }
            break;
        }
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        m_turret.TrackTarget(false);
        m_turret.stop();
        m_shooter.SlowShootingMotors();  // also stops feeder 
        m_shooter.ResetControlType();
        m_shooter.stopRotate();
        m_intake.stopIntake();
        m_shooter.ShotTaken();

        CommandScheduler.getInstance().schedule(new ShotTableShootCommand(m_shooter,m_intake,m_limelight));
    }
}