package frc.robot.util;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.commands.*;
import frc.robot.common.control.Trajectory;
import frc.robot.common.math.RigidTransform2;
import frc.robot.common.math.Rotation2;

public class AutonomousChooser {
    private final AutonomousTrajectories trajectories;

    private static SendableChooser<AutonomousMode> autonomousModeChooser;

    static {
        ShuffleboardTab autoTab = Shuffleboard.getTab("Autonomous settings");

        autonomousModeChooser = new SendableChooser<>();
        autonomousModeChooser.setDefaultOption("Wall Transit", AutonomousMode.W_TRANSIT);
        autonomousModeChooser.addOption("Center Transit", AutonomousMode.C_TRANSIT);
        autonomousModeChooser.addOption("Edge Transit", AutonomousMode.E_TRANSIT);

        // autonomousModeChooser.addOption("Human Player Wall to Friendly 1",
        // AutonomousMode.HP_W_F1);
        // autonomousModeChooser.addOption("Human Player Wall to Friendly 2",
        // AutonomousMode.HP_W_F2);

        // autonomousModeChooser.addOption("Climb Station Wall to Friendly 4",
        // AutonomousMode.CL_W_F4);
        autonomousModeChooser.addOption("Human Player Edge to Friendly 1", AutonomousMode.HP_E_F1);
        autoTab.add("Mode", autonomousModeChooser)
                .withSize(5, 1);
    }

    public AutonomousChooser(AutonomousTrajectories trajectories) {
        this.trajectories = trajectories;
    }

    private SequentialCommandGroup get_W_Transit_Auto(RobotContainer container) {
        SequentialCommandGroup command = new SequentialCommandGroup();

        // reset robot pose
        resetRobotPose(command, container, trajectories.get_Wall_Transit_Trajectory());
        // follow trajectory
        // follow(command, container, trajectories.get_Wall_Transit_Trajectory());

        turnToAngle(command, container, 45);

        return command;
    }

    private SequentialCommandGroup get_E_Transit_Auto(RobotContainer container) {
        SequentialCommandGroup command = new SequentialCommandGroup();

        // reset robot pose
        resetRobotPose(command, container, trajectories.get_Edge_Transit_Trajectory());
        // follow trajectory
        follow(command, container, trajectories.get_Edge_Transit_Trajectory());

        return command;
    }

    private SequentialCommandGroup get_C_Transit_Auto(RobotContainer container) {
        SequentialCommandGroup command = new SequentialCommandGroup();

        // reset robot pose
        resetRobotPose(command, container, trajectories.get_Center_Transit_Trajectory());
        // follow trajectory
        follow(command, container, trajectories.get_Center_Transit_Trajectory());

        return command;
    }

    private SequentialCommandGroup get_HP_W_F1_Auto(RobotContainer container) {
        SequentialCommandGroup command = new SequentialCommandGroup();

        // reset robot pose
        resetRobotPose(command, container, trajectories.get_HP_W_F1_Trajectory());

        // follow trajectory and grab the ball
        followAndIntake(command, container, trajectories.get_HP_W_F1_Trajectory());

        // shoot both balls
        shootAtTarget(command, container);

        return command;
    }

    private SequentialCommandGroup get_HP_W_F2_Auto(RobotContainer container) {
        SequentialCommandGroup command = new SequentialCommandGroup();

        // reset robot pose
        resetRobotPose(command, container, trajectories.get_HP_W_F2_Trajectory());

        // follow trajectory and grab the ball
        followAndIntake(command, container, trajectories.get_HP_W_F2_Trajectory());

        // shoot both balls
        shootAtTarget(command, container);

        return command;
    }

    private SequentialCommandGroup get_CL_W_F4_Auto(RobotContainer container) {
        SequentialCommandGroup command = new SequentialCommandGroup();

        // reset robot pose
        resetRobotPose(command, container, trajectories.get_CL_W_F4_Trajectory());

        // follow trajectory and grab the ball
        followAndIntake(command, container, trajectories.get_CL_W_F4_Trajectory());

        // shoot both balls
        shootAtTarget(command, container);

        return command;
    }

    private SequentialCommandGroup get_HP_E_F1_Auto(RobotContainer container) {
        SequentialCommandGroup command = new SequentialCommandGroup();

        // reset robot pose
        resetRobotPose(command, container, trajectories.get_HP_E_F1_Trajectory());

        // follow trajectory and grab the ball
        followAndIntake(command, container, trajectories.get_HP_E_F1_Trajectory());

        // reset robot pose
        // resetRobotPose(command, container, trajectories.getBackTrajectory());

        follow(command, container, trajectories.getBackTrajectory());

        // shoot both balls
        shootAtTarget(command, container);

        return command;
    }

    public Command getCommand(RobotContainer container) {
        switch (autonomousModeChooser.getSelected()) {
            case W_TRANSIT:
                return get_W_Transit_Auto(container);
            case C_TRANSIT:
                return get_C_Transit_Auto(container);
            case E_TRANSIT:
                return get_E_Transit_Auto(container);
            case HP_W_F1:
                return get_HP_W_F1_Auto(container);
            case HP_W_F2:
                return get_HP_W_F2_Auto(container);
            case CL_W_F4:
                return get_CL_W_F4_Auto(container);
            case HP_E_F1:
                return get_HP_E_F1_Auto(container);
        }

        return get_W_Transit_Auto(container);
    }

    private void follow(SequentialCommandGroup command, RobotContainer container, Trajectory trajectory) {

        Command followCommand = new FollowTrajectoryCommand(container.getDrivetrainSubsystem(), trajectory);

        if (container.getTurretSubsystem().hasHomed()) {
            command.addCommands(followCommand);
        } else {
            Command turretResetCommand = new zTurretResetCommand(container.getTurretSubsystem(),
                    Constants.turretInitSpeed,
                    Constants.turretHomeSpeed, Constants.turretHomePos);
            command.addCommands(followCommand.alongWith(turretResetCommand));
        }
    }

    private void followAndIntake(SequentialCommandGroup command, RobotContainer container, Trajectory trajectory) {
        command.addCommands(new InstantCommand(() -> container.getPneumaticsSubsystem().setIntakeArms(true)));

        command.addCommands(
                new FollowTrajectoryCommand(container.getDrivetrainSubsystem(), trajectory)
                        .deadlineWith(
                                new IntakeCommand(
                                        container.getIntakeSubsystems()[0],
                                        Constants.intakeSpeed)
                                                .alongWith(new IntakeCommand(container.getIntakeSubsystems()[1],
                                                        Constants.intakeInnerSpeed))));

        command.addCommands(new InstantCommand(() -> container.getPneumaticsSubsystem().setIntakeArms(false)));
    }

    private void shootAtTarget(SequentialCommandGroup command, RobotContainer container) {
        shootAtTarget(command, container, 3.5);
    }

    private void turnToAngle(SequentialCommandGroup command, RobotContainer container, double Angle) {
        command.addCommands(new TurnToAngleCommand(container.getDrivetrainSubsystem(), Angle));
    }

    private void shootAtTarget(SequentialCommandGroup command, RobotContainer container, double timeToWait) {
        command.addCommands(
                new zTurretLimelightCommand(container.getTurretSubsystem(), container.getVisionSubsystem(),
                        Constants.turretSpeed, Constants.turretMinSpeed,
                        Constants.LimeLightValues.targetXPosRange,
                        Constants.TurretTargetRange)
                                .alongWith(
                                        new CheckAimAlignmentCommand(container.getVisionSubsystem(),
                                                container.getTurretSubsystem())
                                                        .andThen(
                                                                new zSpinLoadShootDistanceCommand(
                                                                        container.getShooterSubsystem(),
                                                                        container.getConveyorSubsystem(),
                                                                        container.getFeederSubsystem(),
                                                                        container.getVisionSubsystem())
                                                                                .alongWith(new IntakeCommand(container
                                                                                        .getIntakeSubsystems()[1],
                                                                                        Constants.intakeInnerSpeed))))
                                .withTimeout(timeToWait));
    }

    private void resetRobotPose(SequentialCommandGroup command, RobotContainer container, Trajectory trajectory) {
        // command.addCommands(
        // new InstantCommand(() ->
        // container.getDrivetrainSubsystem().resetGyroAngle(Rotation2.ZERO)));
        // command.addCommands(new InstantCommand(() ->
        // container.getDrivetrainSubsystem().resetPose(
        // new RigidTransform2(trajectory.calculate(0.0).getPathState().getPosition(),
        // Rotation2.ZERO))));
    }

    private enum AutonomousMode {
        W_TRANSIT,
        C_TRANSIT,
        E_TRANSIT,
        HP_W_F1,
        HP_W_F2,
        CL_W_F4,
        HP_E_F1,
    }
}
