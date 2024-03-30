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
import frc.robot.Constants;
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

public class ShooterNotePass extends Command {
    private final ShooterSubsystem m_shooter;
    private final TurretSubsystem m_turret;
    private final IntakeSubsystem m_intake;
    private final Timer m_timer = new Timer();
    private boolean m_isRunning=false;
    private boolean m_shouldPrime=false;
    private double m_startTime;
    private double kTimeToRun=Constants.ShooterConstants.timeToRunShooter;
    private double m_targetAngle=0;
    private double m_motor1Veloc=0;
    private double m_motor2Veloc=0;

    private enum STATE{
        PRIMEMOTORS,SETANGLE,WAITFORANGLE,RAMPUPSHOOTER,SHOOT,END
    }

    private STATE m_state=STATE.SETANGLE;

    public ShooterNotePass(ShooterSubsystem shooter,TurretSubsystem turret,IntakeSubsystem intake,double motor1Veloc,double motor2Veloc,double angle,boolean shouldPrime) {
        m_shooter = shooter;
        m_turret = turret;
        m_intake=intake;
        m_shouldPrime=shouldPrime;
        m_motor1Veloc=motor1Veloc;
        m_motor2Veloc=motor2Veloc;
        m_targetAngle=angle;
        addRequirements(shooter,intake);
    }


    @Override
    public void initialize() {
        m_shooter.SlowShootingMotors();
        m_timer.reset();
        m_timer.start();
        m_startTime=Timer.getFPGATimestamp();
        m_state=STATE.PRIMEMOTORS;
        m_isRunning=true;
    }
  
    @Override
    public boolean isFinished(){
        boolean returnValue=false;
        double angleGap=0;
        switch(m_state){
            case PRIMEMOTORS:
                m_shooter.RunShootingMotors(m_motor1Veloc,m_motor2Veloc,false);
                m_state=STATE.SETANGLE;
            break;        
            case SETANGLE: 
                m_shooter.setAngle(m_targetAngle);
                m_state=STATE.WAITFORANGLE;
            break;
            case WAITFORANGLE:
                angleGap=m_shooter.getRelativePosition()
                    -m_targetAngle;
                if(angleGap<Constants.ShooterConstants.kAngleDeadband&&angleGap>-Constants.ShooterConstants.kAngleDeadband){
                    m_shooter.stopRotate();
                    if(!m_shouldPrime){
                        m_startTime=Timer.getFPGATimestamp();
                        m_state=STATE.RAMPUPSHOOTER;
                    }else{
                        m_state=STATE.END; 
                    }
                }
            break;
            case RAMPUPSHOOTER:                   
                if(m_shooter.isMotorVelocitysAtDesiredSpeed(m_motor1Veloc,m_motor2Veloc)){
                m_state=STATE.SHOOT;
                }
            break;
            case SHOOT:           
                m_intake.runIntake(Constants.IntakeConstants.autoIntakeSpeed);
                m_shooter.RunFeedRollers(ShooterConstants.feederSpeed);
                     
            if(m_startTime+kTimeToRun<Timer.getFPGATimestamp()){
                m_state=STATE.END;
            }
            break;
            case END:
                returnValue=true;
        }
        return returnValue;
    }

    @Override
    public void end(boolean interrupted) {
        m_turret.stop();
        if(!m_shouldPrime){
            m_shooter.SlowShootingMotors();
        } 
        m_shooter.ResetControlType();
        m_shooter.stopRotate();
        m_intake.stopIntake();
        m_intake.ResetToggleBoolean();
        m_isRunning=false;
        m_state=STATE.PRIMEMOTORS;
        m_shooter.ShotTaken();
    }

}