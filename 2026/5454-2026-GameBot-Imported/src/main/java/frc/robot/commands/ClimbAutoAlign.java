// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.utilities.LimelightHelpers;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class ClimbAutoAlign extends Command {
  private PIDController xController, yController, rotController;
  private boolean isRightScore;
  private Timer dontSeeTagTimer, stopTimer;
  private CommandSwerveDrivetrain drivebase;
  private double tagID = 31;

  public ClimbAutoAlign(boolean isRightScore, CommandSwerveDrivetrain drivebase) {
    xController = new PIDController(Constants.ClimbConstants.AutoAlign.climbYpos, 0.0, 0);  // Vertical movement
    yController = new PIDController(Constants.ClimbConstants.AutoAlign.climbXpos, 0.0, 0);  // Horitontal movement
    rotController = new PIDController(Constants.ClimbConstants.AutoAlign.alignRotation, 0, 0);  // Rotation
    this.isRightScore = isRightScore;
    this.drivebase = drivebase;
    addRequirements(drivebase);
  }

  @Override
  public void initialize() {
    this.stopTimer = new Timer();
    this.stopTimer.start();
    this.dontSeeTagTimer = new Timer();
    this.dontSeeTagTimer.start();

    rotController.setSetpoint(Constants.ClimbConstants.AutoAlign.rotSetpoint);
    rotController.setTolerance(Constants.ClimbConstants.AutoAlign.rotTolerance);

    xController.setSetpoint(Constants.ClimbConstants.AutoAlign.xSetpoint);
    xController.setTolerance(Constants.ClimbConstants.AutoAlign.xTolerance);

    yController.setSetpoint(isRightScore ? Constants.ClimbConstants.AutoAlign.ySetpoint : -Constants.ClimbConstants.AutoAlign.ySetpoint);
    yController.setTolerance(Constants.ClimbConstants.AutoAlign.yTolerance);

    tagID = LimelightHelpers.getFiducialID("");
  }

  @Override
  public void execute() {
    if (LimelightHelpers.getTV("") && LimelightHelpers.getFiducialID("") == tagID) {
      this.dontSeeTagTimer.reset();

      double[] postions = LimelightHelpers.getBotPose_TargetSpace("");
      SmartDashboard.putNumber("x", postions[2]);

      double xSpeed = xController.calculate(postions[2]);
      SmartDashboard.putNumber("xspee", xSpeed);
      double ySpeed = -yController.calculate(postions[0]);
      double rotValue = -rotController.calculate(postions[4]);

      drivebase.drive(xSpeed, ySpeed, rotValue);

      if (!rotController.atSetpoint() ||
          !yController.atSetpoint() ||
          !xController.atSetpoint()) {
        stopTimer.reset();
      }
    } else {
      drivebase.drive(0, 0, 0);
    }

    SmartDashboard.putNumber("poseValidTimer", stopTimer.get());
  }

  @Override
  public void end(boolean interrupted) {
    drivebase.drive(0, 0, 0);
  }

  @Override
  public boolean isFinished() {
    // Requires the robot to stay in the correct position for 0.3 seconds, as long as it gets a tag in the camera
    return this.dontSeeTagTimer.hasElapsed(Constants.ClimbConstants.AutoAlign.waitTime) ||
        stopTimer.hasElapsed(Constants.ClimbConstants.AutoAlign.validationTime);
  }
}