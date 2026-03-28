package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.utilities.Limelight;

public class ResetGyroCommand extends Command{
    private CommandSwerveDrivetrain m_swerve;
    private Limelight m_limelight;
    
    public ResetGyroCommand(CommandSwerveDrivetrain swerve, Limelight limelight){
        m_swerve=swerve;
        m_limelight=limelight;
        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        m_swerve.gyroReset();
        m_limelight.SetIMUMode(0);
    }

    @Override
    public void end(boolean interrupted){
    }

    @Override
    public boolean isFinished(){
        return true;
    }

}
