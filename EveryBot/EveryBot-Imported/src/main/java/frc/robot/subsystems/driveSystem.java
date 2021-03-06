// RobotBuilder Version: 2.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// Java from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.


package frc.robot.subsystems;


import frc.robot.commands.*;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

// END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS


/**
 *
 */
public class driveSystem extends Subsystem {

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTANTS

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTANTS

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
private WPI_VictorSPX leftDriveFront;
private WPI_VictorSPX leftDriveBack;
private SpeedControllerGroup leftDriveGroup;
private WPI_VictorSPX rightDriveFront;
private WPI_VictorSPX rightDriveBack;
private SpeedControllerGroup rightDriveGroup;
private DifferentialDrive differentialDrive;

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS

    public driveSystem() {
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS
leftDriveFront = new WPI_VictorSPX(1);


        
leftDriveBack = new WPI_VictorSPX(2);


        
leftDriveGroup = new SpeedControllerGroup(leftDriveFront, leftDriveBack  );
addChild("leftDriveGroup",leftDriveGroup);

        
rightDriveFront = new WPI_VictorSPX(3);


        
rightDriveBack = new WPI_VictorSPX(4);


        
rightDriveGroup = new SpeedControllerGroup(rightDriveFront, rightDriveBack  );
addChild("rightDriveGroup",rightDriveGroup);

        
differentialDrive = new DifferentialDrive(leftDriveGroup, rightDriveGroup);
addChild("differentialDrive",differentialDrive);
differentialDrive.setSafetyEnabled(true);
differentialDrive.setExpiration(0.1);
differentialDrive.setMaxOutput(1.0);

        

        // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS
    }

    @Override
    public void initDefaultCommand() {
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DEFAULT_COMMAND


        // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DEFAULT_COMMAND

        // Set the default command for a subsystem here.
        // setDefaultCommand(new MySpecialCommand());
    }

    @Override
    public void periodic() {
        // Put code here to be run every loop

    }

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CMDPIDGETTERS


    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CMDPIDGETTERS

    // Put methods for controlling this subsystem
    // here. Call these from Commands.

}

