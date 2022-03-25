package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.common.math.Vector2;

public class DriveCommand extends CommandBase {
    private DrivetrainSubsystem drivetrainSubsystem;
    private Double forward;
    private Double strafe;
    private Double rotation;

    public DriveCommand(DrivetrainSubsystem drivetrain, Double forward, Double strafe, Double rotation) {
        this.forward = forward;
        this.strafe = strafe;
        this.rotation = rotation;

        drivetrainSubsystem = drivetrain;

        addRequirements(drivetrain);
    }

    @Override
    public void execute() {
        drivetrainSubsystem.drive(new Vector2(forward, strafe), rotation, true);
    }

    @Override
    public void end(boolean interrupted) {
        drivetrainSubsystem.drive(Vector2.ZERO, 0, false);
    }
}
