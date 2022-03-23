package frc.robot;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.util.AutonomousChooser;
import frc.robot.util.AutonomousTrajectories;
import frc.robot.common.input.Axis;
import frc.robot.common.input.XboxController;

import java.io.IOException;

public class RobotContainer {
  private final XboxController primaryController = new XboxController(0);

  private final DrivetrainSubsystem drivetrainSubsystem = new DrivetrainSubsystem();
  private final VisionSubsystem visionSubsystem = new VisionSubsystem(drivetrainSubsystem);

  private AutonomousTrajectories autonomousTrajectories;
  private final AutonomousChooser autonomousChooser;

  public RobotContainer() {
    try {
      autonomousTrajectories = new AutonomousTrajectories(DrivetrainSubsystem.TRAJECTORY_CONSTRAINTS);
    } catch (IOException e) {
      e.printStackTrace();
    }
    autonomousChooser = new AutonomousChooser(autonomousTrajectories);
    ;

    CommandScheduler.getInstance().setDefaultCommand(drivetrainSubsystem,
        new DriveCommand(drivetrainSubsystem, getDriveForwardAxis(), getDriveStrafeAxis(), getDriveRotationAxis()));
    CommandScheduler.getInstance().registerSubsystem(visionSubsystem);

    configureButtonBindings();
  }

  private void configureButtonBindings() {
  }

  public Command getAutonomousCommand() {
    return autonomousChooser.getCommand(this);
  }

  private Axis getDriveForwardAxis() {
    return primaryController.getLeftYAxis();
  }

  private Axis getDriveStrafeAxis() {
    return primaryController.getLeftXAxis();
  }

  private Axis getDriveRotationAxis() {
    return primaryController.getRightXAxis();
  }

  public VisionSubsystem getVisionSubsystem() {
    return visionSubsystem;
  }

  public DrivetrainSubsystem getDrivetrainSubsystem() {
    return drivetrainSubsystem;
  }
}
