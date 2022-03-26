// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.Timer;

/** An example command that uses an example subsystem. */
public class zIntakeTimeCommand extends CommandBase {
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
    private final IntakeSubsystem m_IntakeSubsystem;
    private final IntakeSubsystem m_InnerIntakeSubsystem;
    private final double m_speed;
    private final double m_duration;
    private final boolean m_keepRunning;
    private boolean m_isFinished = false;

    /**
     * Creates a new ExampleCommand.
     *
     * @param subsystem
     *            The subsystem used by this command.
     */
    public zIntakeTimeCommand(IntakeSubsystem intake, IntakeSubsystem innerIntake, double speed, double duration,
            boolean keepRunning) {
        m_IntakeSubsystem = intake;
        m_InnerIntakeSubsystem = innerIntake;
        m_speed = speed;
        m_duration = duration;
        m_keepRunning = keepRunning;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(m_IntakeSubsystem);
    }

    public zIntakeTimeCommand(IntakeSubsystem intake, IntakeSubsystem innerIntake, double speed, double duration) {
        m_IntakeSubsystem = intake;
        m_InnerIntakeSubsystem = innerIntake;
        m_speed = speed;
        m_duration = duration;
        m_keepRunning = false;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(m_IntakeSubsystem);
        addRequirements(m_InnerIntakeSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double startTime = Timer.getFPGATimestamp();
        double currentTime = Timer.getFPGATimestamp();
        startTime = Timer.getFPGATimestamp();
        do {
            m_IntakeSubsystem.runIntake(m_speed);
            m_InnerIntakeSubsystem.runIntake(m_speed);
            currentTime = Timer.getFPGATimestamp();
            System.out.println(startTime + " - " + currentTime);
        } while (currentTime < startTime + m_duration);
        m_isFinished = true;
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        if (m_keepRunning == false) {
            m_IntakeSubsystem.stopIntake();
            m_InnerIntakeSubsystem.stopIntake();
        }
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return m_isFinished;
    }
}
