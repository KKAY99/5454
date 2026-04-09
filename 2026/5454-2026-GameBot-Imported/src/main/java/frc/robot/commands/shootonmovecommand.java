package frc.robot.commands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.shooter.NewShooterSubsystem;
import frc.robot.subsystems.shooter.TurretUtil;
import frc.robot.subsystems.shooter.TurretUtil.ShotSolution;
import frc.robot.subsystems.shooter.TurretUtil.TargetType;


public class shootonmovecommand extends Command {

    private final CommandSwerveDrivetrain m_swerve;
    private final NewShooterSubsystem m_shooter;

    public shootonmovecommand(CommandSwerveDrivetrain drivetrain, NewShooterSubsystem shooter) {
        m_swerve = drivetrain;
        m_shooter = shooter;
        addRequirements(shooter); 
    }

    @Override
    public void execute() {
        Pose2d robotPose = m_swerve.getPose2d();
        ChassisSpeeds fieldSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(
            m_swerve.getChassisSpeeds(),
            robotPose.getRotation()
        );

        ShotSolution solution = TurretUtil.computeLeadShotSolution(
            robotPose,
            fieldSpeeds.vxMetersPerSecond,
            fieldSpeeds.vyMetersPerSecond,
            TargetType.HUB
        );

        if (solution.isValid()) {
            m_shooter.setTurretAngle(solution.turretAngle);
            m_shooter.setHoodPosition(solution.hoodPosition);
            m_shooter.setShooterSpeed(solution.shooterSpeed);
        }
    }
}