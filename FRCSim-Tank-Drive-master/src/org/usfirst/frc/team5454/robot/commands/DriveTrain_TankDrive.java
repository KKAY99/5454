package org.usfirst.frc.team5454.robot.commands;

import org.usfirst.frc.team5454.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class DriveTrain_TankDrive extends Command {
	
    private final double speedScale = 1.0;
    
    private final double deadzone = 0.07;

    public DriveTrain_TankDrive() {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	requires(Robot.driveTrain);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	Robot.driveTrain.setMotors(0, 0);
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	// Scaling joysticks to limit max speed
    	double leftPower = Robot.oi.joystickLeft.getY() * speedScale;
        double rightPower = Robot.oi.joystickRight.getY() * speedScale;
        
        // Adding a range of the joysticks in which the robot will not respond
        leftPower = (Math.abs(leftPower) < deadzone)? 0 : leftPower;
        rightPower = (Math.abs(rightPower) < deadzone)? 0 : rightPower;
        
        Robot.driveTrain.setMotors(leftPower, rightPower);
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return false;
    }

    // Called once after isFinished returns true
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
