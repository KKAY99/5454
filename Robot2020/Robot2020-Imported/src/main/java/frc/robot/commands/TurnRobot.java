package frc.robot.commands;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class TurnRobot extends CommandBase {
    private final DriveSubsystem m_drive;
    private final double m_speed;
    private final double m_angle;
    /**
     * Creates a TurnRobot
     *
     * @param subsystem The drive subsystem this command wil run on.
     * @param speed The speed is how fast robot shot execute turn
     * @param angle The angle amount the robot should turn (positive to right, negative to left)
     */
    public TurnRobot(DriveSubsystem subsystem, double speed , double angle) {
      m_drive = subsystem;
      m_speed = speed;
      m_angle = angle;
      addRequirements(m_drive);
    }
  
    @Override
    public void execute() {
     // System.out.println('Speeds' +  m_left.getAsDouble()  + m_right.getAsDouble()
      m_drive.commandTurnRobot(m_speed, m_angle);
    }
}

