package frc.robot.commands;
 
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ClimbConstants;
import frc.robot.subsystems.ClimbSubsystem;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.utilities.Limelight;

public class ClimbDownCommand extends Command{
    private ClimbSubsystem m_climb;
    private double m_climbSpeed;
    public ClimbDownCommand(ClimbSubsystem climb,double speed){
        m_climb=climb;
        m_climbSpeed=speed;
        addRequirements(climb);
    }

    @Override
    public void initialize() {
                m_climb.setClimbSpeed(m_climbSpeed);

    }

    @Override
    public void end(boolean interrupted){
        m_climb.stopClimb();
    }

    @Override
    public boolean isFinished(){
        boolean returnValue=false;
        
    if (m_climb.isClimbDownLimit()) {
      m_climb.stopClimb();
      returnValue=true;
    } 
    return returnValue;    
    }
}