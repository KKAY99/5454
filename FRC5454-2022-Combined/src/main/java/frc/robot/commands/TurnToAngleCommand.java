package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.common.math.Vector2;
import frc.robot.Constants;

public class TurnToAngleCommand extends CommandBase {

    private final DrivetrainSubsystem m_drive;
    private final double m_TargetAngle;
    private final PIDController turnPID;

    private final double kP = 0.0005;
    private final double kI = 0.0;
    private final double kD = 0.0;

    /**
     * Creates a new DefaultDrive.
     *
     * @param subsystem
     *                  The drive subsystem this command wil run on.
     * @param forward
     *                  The control input for driving left
     * @param rotation
     *                  The control input for driving right
     */
    public TurnToAngleCommand(DrivetrainSubsystem subsystem, double Angle) {
        m_drive = subsystem;
        m_TargetAngle = Angle;
        turnPID = new PIDController(kP, kI, kD);
        turnPID.enableContinuousInput(0.0, 360.0);
        addRequirements(m_drive);
    }

    @Override
    public void execute() {

        m_drive.drive(
                new Vector2(0.0, 0.0),
                turnPID.calculate(
                        m_drive.getPose().rotation.toDegrees(),
                        m_TargetAngle),
                true);

    }

    @Override
    public boolean isFinished() {
        double currentAngle = m_drive.getPose().rotation.toDegrees();

        boolean isAlinged = currentAngle < m_TargetAngle + 2.0
                && currentAngle > m_TargetAngle - 2.0;

        System.out.println("Current Angle: " + currentAngle);
        System.out.println("Target Angle: " + m_TargetAngle);
        System.out.println("isAligned: " + isAlinged);

        double powerOutput = turnPID.calculate(
            m_drive.getPose().rotation.toDegrees(),
            m_TargetAngle);

        System.out.println(powerOutput);

        boolean lowOutput = Math.abs(powerOutput) < 0.1;

        return isAlinged && lowOutput;
    }
}