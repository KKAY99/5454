// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.sql.Driver;
import java.util.List;

import javax.swing.JToggleButton;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants.InputControllers;
import frc.robot.Constants.AutoConstants;
import frc.robot.commands.*;
import frc.robot.subsystems.*;



/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // The robot's subsystems and commands are defined here...
    
    private Swerve m_swerve = new Swerve();
    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;
 
    private XboxController m_xBoxDriver = new XboxController(InputControllers.kXboxDrive);
    private SendableChooser<String> m_autoChooser = new SendableChooser<>(); 
    private SendableChooser<AutoConstants.StartingLocations> m_autoStart = new SendableChooser<>(); 
    private SendableChooser<Double> m_autoDelay = new SendableChooser<>(); 
    private SendableChooser<String> m_autoChosen = new SendableChooser<>(); 
    
    public RobotContainer(){
      // Configure the button bindings
        configureButtonBindings();
        //Create Auto Commands
        createAutonomousCommandList();
        m_swerve.setDefaultCommand(
        m_swerve.drive(
            () -> m_xBoxDriver.getRawAxis(translationAxis),
            () -> m_xBoxDriver.getRawAxis(strafeAxis),
            () -> m_xBoxDriver.getRawAxis(rotationAxis)));

    }
       
    
    /*m*
     * Use this method to define your button->command mappings. Buttons can be
     * created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
     * it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings(){
    
    }
       
    public void refreshSmartDashboard(){  
      m_swerve.getPose();
    }
    
  

   /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */

  private void createAutonomousCommandList(){
      m_autoChooser.setDefaultOption(AutoConstants.autoMode0,AutoConstants.autoMode0);
      m_autoChooser.addOption(AutoConstants.autoMode1,AutoConstants.autoMode1);
      SmartDashboard.putData("Auto Chooser",m_autoChooser);
  }

  public Command getAutonomousCommand(){
    Command newCommand=null;

    return newCommand;
  }

  
  public void AutonMode(){
    homeRobot();  
  }  
  public void TeleopMode(){
    homeRobot();
    
  }
  
  private void homeRobot(){
  
  }    
}