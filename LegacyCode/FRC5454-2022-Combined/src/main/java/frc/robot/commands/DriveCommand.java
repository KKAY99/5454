package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.Constants;
import frc.robot.common.Utilities;
import frc.robot.common.math.Vector2;

public class DriveCommand extends CommandBase {
    private DrivetrainSubsystem drivetrainSubsystem;
    private DoubleSupplier m_drive_fwd;
    private DoubleSupplier m_drive_strafe;
    private DoubleSupplier m_drive_rcw;

    public DriveCommand(DrivetrainSubsystem drivetrain, DoubleSupplier forward, DoubleSupplier strafe,
            DoubleSupplier rotation) {
        this.m_drive_fwd = forward;
        this.m_drive_strafe = strafe;
        this.m_drive_rcw = rotation;

        drivetrainSubsystem = drivetrain;

        addRequirements(drivetrain);
    }

    // @Override
    // public void execute() {
    // drivetrainSubsystem.drive(new Vector2(forward.getAsDouble(),
    // strafe.getAsDouble()),
    // rotation.getAsDouble(),
    // true);
    // }

    @Override
    public void execute() {
        double forward = Utilities.deadband(m_drive_fwd.getAsDouble(), Constants.driveDeadband);
        // Square the forward stick
        forward = Math.copySign(Math.pow(forward, 2.0), forward);

        double strafe = Utilities.deadband(m_drive_strafe.getAsDouble());
        strafe = Utilities.deadband(strafe, Constants.driveDeadband);
        // Square the strafe stick
        strafe = Math.copySign(Math.pow(strafe, 2.0), strafe);
        // System.out.println("Straffing - " + strafe);
        double rotation = Utilities.deadband(m_drive_rcw.getAsDouble());
        rotation = Utilities.deadband(rotation, Constants.driveDeadband);
        // Square the rotation stick
        rotation = Math.copySign(Math.pow(rotation, 2.0), rotation);
        // System.out.println(forward + " -- " + strafe + " -- " + rotation);
        drivetrainSubsystem.drive(new Vector2(forward, strafe), rotation, true);

    }

    @Override
    public void end(boolean interrupted) {
        drivetrainSubsystem.drive(Vector2.ZERO, 0, false);
    }
}
