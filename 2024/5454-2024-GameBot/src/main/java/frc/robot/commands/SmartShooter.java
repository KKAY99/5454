package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.SmartShooterConstants;
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
    private final boolean m_updatePose;
    private final Timer m_timer = new Timer();
    private ShotTable m_shotTable = new ShotTable();
   
    public SmartShooter(ShooterSubsystem shooter, TurretSubsystem turret, Swerve drive, boolean updatePose) {
        m_shooter = shooter;
        m_turret = turret;
        m_drive = drive;
        m_updatePose = updatePose;
        addRequirements(shooter, turret);
    }


    @Override
    public void initialize() {
        m_turret.TrackTarget(true);
        m_timer.reset();
        m_timer.start();
    }

    @Override
    public void execute() {

        double currentTime = m_timer.get();
 
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
        double dist = robotToGoal.getDistance(new Translation2d()) * 39.37; //WHY 39.37

        SmartDashboard.putNumber("Calculated (in)", dist);

        //double fixedShotTime = m_timeTable.getOutput(dist);
        double shotTime = m_shotTable.getShotTime(dist);
    

        SmartDashboard.putNumber("Fixed Time", shotTime);

        Translation2d movingGoalLocation = new Translation2d();
/*      
        //loop throuh to allow for converge
        for(int i=0;i<5;i++){

            double virtualGoalX = target.getX()
                    - shotTime * (robotVel.vx + robotAccel.ax * ShooterConstants.kAccelCompFactor);
            double virtualGoalY = target.getY()
                    - shotTime * (robotVel.vy + robotAccel.ay * ShooterConstants.kAccelCompFactor);

            SmartDashboard.putNumber("Goal X", virtualGoalX);
            SmartDashboard.putNumber("Goal Y", virtualGoalY);

            Translation2d testGoalLocation = new Translation2d(virtualGoalX, virtualGoalY);

            Translation2d toTestGoal = testGoalLocation.minus(m_drive.getPose().getTranslation());
 
            double newShotTime = m_timeTable.getOutput(toTestGoal.getDistance(new Translation2d()) * 39.37);

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

        m_turret.aimAtGoal(m_drive.getPose(), movingGoalLocation, false);

        if (SmartDashboard.getBoolean("Adjust Shot?", false)) {
            m_shooter.run(m_rpmTable.getOutput(newDist) + SmartDashboard.getNumber("SetShotAdjust", 0));
            m_hood.run(m_hoodTable.getOutput(newDist) + SmartDashboard.getNumber("SetHoodAdjust", 0));
        } else {
            m_shooter.run(m_rpmTable.getOutput(newDist));
            m_hood.run(m_hoodTable.getOutput(newDist));

        }

        if (currentTime > 0.250 && Limelight.valid() && Limelight.getDistance() >= 85.0) {
            double dL = Limelight.getDistance() * 0.0254;
            double tR = m_drive.getGyro().getRadians();
            double tT = m_turret.getMeasurement() - Math.PI;
            double tL = -1.0 * Limelight.tx();

            Pose2d pose = calcPoseFromVision(dL, tR, tT, tL, GoalConstants.kGoalLocation);

            if (m_updatePose) {
                m_drive.setPose(pose);
            }

        }

 */
    }

    @Override
    public void end(boolean interrupted) {
        SmartDashboard.putBoolean("Shooter Running", false);
        m_turret.TrackTarget(false);
        m_turret.stop();
        m_shooter.StopShootingMotors();
        m_timer.stop();
    }

    private Pose2d calcPoseFromVision(double dL, double tR, double tT, double tL, Translation2d goal) {
        double tG = tR + tT + tL;
        double rX = goal.getX() - dL * Math.cos(tG);
        double rY = goal.getY() - dL * Math.sin(tG);

        return new Pose2d(rX, rY, new Rotation2d(-tR));
    }
}