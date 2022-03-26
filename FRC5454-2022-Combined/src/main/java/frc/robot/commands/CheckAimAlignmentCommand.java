package frc.robot.commands;

import frc.robot.classes.Limelight;
import frc.robot.subsystems.TurretSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class CheckAimAlignmentCommand extends CommandBase {
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
    private final Limelight m_VisionSubsystem;
    private final TurretSubsystem m_TurretSubsystem;

    public CheckAimAlignmentCommand(Limelight visionSubsystem, TurretSubsystem TurretSubsystem) {
        m_VisionSubsystem = visionSubsystem;
        m_TurretSubsystem = TurretSubsystem;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        // TODO: Check this value
        return m_TurretSubsystem.hasHomed() && m_VisionSubsystem.isOnTargetX() && m_TurretSubsystem.getPower() < 0.05;
    }
}
