package org.usfirst.frc.team5454.robot.subsystems;

import org.usfirst.frc.team5454.robot.RobotMap;
import org.usfirst.frc.team5454.robot.commands.DriveTrain_TankDrive;
import edu.wpi.first.wpilibj.SpeedController.Talon;

import edu.wpi.first.wpilibj.command.Subsystem;

/**
 *
 */
public class DriveTrain extends Subsystem {
	
    Talon leftMotor1 = new Talon(RobotMap.DRIVETRAIN_TALON_LEFT1);
    Talon leftMotor2 = new Talon(RobotMap.DRIVETRAIN_TALON_LEFT2);
    Talon rightMotor1 = new Talon(RobotMap.DRIVETRAIN_TALON_RIGHT1);
    Talon rightMotor2 = new Talon(RobotMap.DRIVETRAIN_TALON_RIGHT2)

    // Put methods for controlling this subsystem
    // here. Call these from Commands.

    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        setDefaultCommand(new DriveTrain_TankDrive());
    }
	
	public void setMotors(double left, double right) {
    	left = scaleLeft(left);
    	right = scaleRight(right);
    	
    	setMotorsRaw(left, right);
    }
    
    public void setMotorsRaw(double left, double right) {
    	left = safetyTest(left);
    	right = safetyTest(right);
    	
        leftMotor1.set(left);
        leftMotor2.set(left);
        rightMotor1.set(right);	
        rightMotor2.set(right);	
	}
    
    private double safetyTest(double motorValue) {
        motorValue = (motorValue < -1) ? -1 : motorValue;
        motorValue = (motorValue > 1) ? 1 : motorValue;
        
        return motorValue;
    }
    
    private double scaleLeft(double left) {
    	return 1.0 * left;
    }
    
    private double scaleRight(double right) {
    	return 1.0 * right;
    }
}

