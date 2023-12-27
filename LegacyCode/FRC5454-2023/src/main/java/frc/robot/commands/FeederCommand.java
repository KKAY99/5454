// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.FeederSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class FeederCommand extends CommandBase {
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
    private final FeederSubsystem m_FeederSubsystem;
    private final double m_speed;

    /**
     * Creates a new ExampleCommand.
     *
     * @param subsystem The subsystem used by this command.
     */
    public FeederCommand(FeederSubsystem feeder,double speed) {
        m_FeederSubsystem = feeder;
        m_speed=speed;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(m_FeederSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        m_FeederSubsystem.run(m_speed);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_FeederSubsystem.stop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
