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

public class SmartShooter extends Command {
    private final ShooterSubsystem m_shooter;
    private final TurretSubsystem m_turret;
    private final Swerve m_drive;
    private final IntakeSubsystem m_intake;
    private final boolean m_updatePose;
    private final Limelight m_limelight;
    private final Timer m_timer = new Timer();
    private ShotTable m_shotTable = new ShotTable();
    private final double kConvertInchestoMeters=39.37;
    private boolean m_isRunning=false;
    private boolean m_shootWhileMove=false;
    private double m_currentTime;
    private double kTimeToRun=Constants.ShooterConstants.timeToRunShooter;
    private double m_feederStartTime;
    private enum STATE{
        SETANGLE,WAITFORANGLE,RAMPUPSHOOTER,SHOOT,END
      }
    private STATE m_state=STATE.SETANGLE;
    public SmartShooter(ShooterSubsystem shooter, TurretSubsystem turret, Swerve drive, Limelight limelight,IntakeSubsystem intake,boolean updatePose,boolean shootwhileMove) {
        m_shooter = shooter;
        m_turret = turret;
        m_drive = drive;
        m_limelight = limelight;
        m_intake=intake;
        m_updatePose = updatePose;
        m_shootWhileMove=shootwhileMove;
        addRequirements(shooter,turret);
    }


    @Override
    public void initialize() {
        m_turret.TrackTarget(true);
        m_timer.reset();
        m_timer.start();
        m_currentTime=Timer.getFPGATimestamp();
        m_state=STATE.SETANGLE;
    }
    private void shootwhileMove(){

        double currentTime = m_timer.get();
        m_isRunning=true;
        Logger.recordOutput("Shooter/SmartShooterCommand",m_isRunning);

        // m_drive.getRobotVelocity().fromFieldRelativeSpeeds(null, null)
       //NEED TO FINISH getGyro2D;
       FieldRelativeSpeed robotVel = m_drive.getRelativeSpeed();
       FieldRelativeAccel robotAccel =m_drive.getRelativeAccel();
        Alliance currentAlliance=DriverStation.getAlliance().get();
        Translation2d target=null;
        if(currentAlliance==Alliance.Red){
            target = SmartShooterConstants.kRedSpeakerLocation;
        } else {
             target = SmartShooterConstants.kRedSpeakerLocation;
        }
        Translation2d robotToGoal = target.minus(m_drive.getPose().getTranslation());
        double dist = robotToGoal.getDistance(new Translation2d()) * kConvertInchestoMeters;
        SmartDashboard.putNumber("Calculated (in)", dist);

        //double fixedShotTime = m_timeTable.getOutput(dist);
        double shotTime = m_shotTable.getShotTime(dist);
    

        SmartDashboard.putNumber("Fixed Time", shotTime);

        Translation2d movingGoalLocation = new Translation2d();
      
       // loop throuh to allow for converge
        for(int i=0;i<5;i++){

            double virtualGoalX = target.getX()
                    - shotTime * (robotVel.vx + robotAccel.ax * ShooterConstants.kAccelCompFactor);
            double virtualGoalY = target.getY()
                    - shotTime * (robotVel.vy + robotAccel.ay * ShooterConstants.kAccelCompFactor);

            SmartDashboard.putNumber("Goal X", virtualGoalX);
            SmartDashboard.putNumber("Goal Y", virtualGoalY);

            Translation2d testGoalLocation = new Translation2d(virtualGoalX, virtualGoalY);

            Translation2d toTestGoal = testGoalLocation.minus(m_drive.getPose().getTranslation());
 
            double newShotTime = m_shotTable.getShotTime(toTestGoal.getDistance(new Translation2d()) * kConvertInchestoMeters);

            if(Math.abs(newShotTime-shotTime) <= 0.010){
                i=4;
            }
            
            if(i == 4){
                movingGoalLocation = testGoalLocation;
                SmartDashboard.putNumber("NewShotTime", newShotTime);
            }
            else{
                shotTime = newShotTime;
            }

        }

        double newDist = movingGoalLocation.minus(m_drive.getPose().getTranslation()).getDistance(new Translation2d()) * 39.37;

        SmartDashboard.putNumber("NewDist", newDist);
        Logger.recordOutput("Shooter/SmartShooterActualDistance",dist);
        Logger.recordOutput("Shooter/SmartShooterNewDistance",newDist);
         

        m_turret.aimAtGoal(m_drive.getPose(), movingGoalLocation, false);
       
        m_shooter.setAngle(m_shotTable.getAngle(newDist));
        m_shooter.RunShootingMotors(m_shotTable.getVelocity(newDist));
     
    
        if (currentTime > 0.250 && m_limelight.isTargetAvailible() && m_limelight.getDistance() >= 85.0) {
            if (m_updatePose) {
                //update pose based on vision
               // m_drive.AddVisionPose(null,0,true,1);            }
            }
        }

 
    }

    private void shootStatic(){
    }

    @Override
    public void execute() {
        if(m_shootWhileMove){
            shootwhileMove();
        }else{
            shootStatic();
        }
        
    }

    @Override
    public boolean isFinished(){
        boolean returnValue=false;
        double limeLimelightDis=m_limelight.getDistance();
        Logger.recordOutput("Shooter/SmartShooterState",m_state.toString());
        switch(m_state){
            case SETANGLE:          
                m_shooter.setAngle(m_shotTable.getAngle(limeLimelightDis));
                m_state=STATE.WAITFORANGLE;
                 break;
            case WAITFORANGLE:
                double angleGap=Math.abs(m_shooter.getRelativePosition())
                    -Math.abs(m_shotTable.getAngle(limeLimelightDis));
                  Logger.recordOutput("Shooter/AngleGap",angleGap);
                if(angleGap<Constants.ShooterConstants.kAngleDeadband && angleGap>-Constants.ShooterConstants.kAngleDeadband ){
                    m_shooter.stopRotate();
                    m_shooter.RunShootingMotors(m_shotTable.getVelocity(limeLimelightDis));
                    m_state=STATE.RAMPUPSHOOTER;
                    m_feederStartTime=Timer.getFPGATimestamp()+Constants.ShooterConstants.kRampUpTime;
                    Logger.recordOutput("Shooter/AngleGap",0);
                }
                 break;
            case RAMPUPSHOOTER:                   
                    if(Timer.getFPGATimestamp()>m_feederStartTime){
                        m_currentTime=Timer.getFPGATimestamp();
                        m_state=STATE.SHOOT;
                    }
                    break;
            case SHOOT:

            
                m_shooter.RunFeedRollers(ShooterConstants.feederSpeed);
                m_intake.runIntake(Constants.IntakeConstants.autoIntakeSpeed);
                     
            if(m_currentTime+kTimeToRun<Timer.getFPGATimestamp()){
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
        m_turret.TrackTarget(false);
        m_turret.stop();
        m_shooter.StopFeedRollers();
        m_shooter.StopShootingMotors();
        m_shooter.ResetControlType();
        m_timer.stop();
        m_intake.stopIntake();
        m_isRunning=false;
        m_state=STATE.SETANGLE;
        m_shooter.ShotTaken();
        Logger.recordOutput("Shooter/ShooterSpeed",0);
        Logger.recordOutput("Shooter/SmartShooterCommand",m_isRunning);
    }

    private Pose2d calcPoseFromVision(double dL, double tR, double tT, double tL, Translation2d goal) {
        double tG = tR + tT + tL;
        double rX = goal.getX() - dL * Math.cos(tG);
        double rY = goal.getY() - dL * Math.sin(tG);

        return new Pose2d(rX, rY, new Rotation2d(-tR));
    }
}